#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#ifdef _WIN32
#include <windows.h>
#include <bcrypt.h>
#define ADAPTER_EXPORT __declspec(dllexport)
#else
#include <openssl/evp.h>
#define ADAPTER_EXPORT __attribute__((visibility("default")))
#endif
#include "HybridExactContact.h"
#include <igl/AABB.h>
#include <igl/readOBJ.h>
#include <igl/signed_distance.h>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <igl/per_edge_normals.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <mutex>
#include <atomic>
#include <cstdlib>

namespace campaign_hybrid {
using Grid=sofa::component::container::DistanceGrid;
std::atomic<unsigned long long> calls{0},coarseRejected{0},exactEvaluations{0},outsideGrid{0};
std::string required(const char*name){const char*s=std::getenv(name);if(!s||!*s)throw std::runtime_error(std::string("Hybrid contract missing ")+name);return s;}
std::string lower(std::string s){std::transform(s.begin(),s.end(),s.begin(),[](unsigned char c){return (char)std::tolower(c);});return s;}
std::string sha256(const std::string&file){
#ifdef _WIN32
 BCRYPT_ALG_HANDLE alg=nullptr;BCRYPT_HASH_HANDLE hash=nullptr;DWORD len=0,got=0;
 if(BCryptOpenAlgorithmProvider(&alg,BCRYPT_SHA256_ALGORITHM,nullptr,0))throw std::runtime_error("SHA256 provider failed");
 if(BCryptGetProperty(alg,BCRYPT_OBJECT_LENGTH,(PUCHAR)&len,sizeof(len),&got,0))throw std::runtime_error("SHA256 property failed");
 std::vector<unsigned char> obj(len),buffer(1<<20),digest(32);
 if(BCryptCreateHash(alg,&hash,obj.data(),len,nullptr,0,0))throw std::runtime_error("SHA256 create failed");
 std::ifstream in(file,std::ios::binary);if(!in)throw std::runtime_error("Hybrid missing file: "+file);
 while(in){in.read((char*)buffer.data(),buffer.size());if(in.gcount()&&BCryptHashData(hash,buffer.data(),(ULONG)in.gcount(),0))throw std::runtime_error("SHA256 update failed");}
 if(BCryptFinishHash(hash,digest.data(),(ULONG)digest.size(),0))throw std::runtime_error("SHA256 finish failed");
 BCryptDestroyHash(hash);BCryptCloseAlgorithmProvider(alg,0);std::ostringstream s;s<<std::hex<<std::setfill('0');for(auto b:digest)s<<std::setw(2)<<(int)b;return s.str();
#else
 std::ifstream in(file,std::ios::binary);if(!in)throw std::runtime_error("Missing file: "+file);
 auto *ctx=EVP_MD_CTX_new();if(!ctx||EVP_DigestInit_ex(ctx,EVP_sha256(),nullptr)!=1)throw std::runtime_error("OpenSSL SHA init failed");
 std::vector<char> buffer(1<<20);while(in){in.read(buffer.data(),buffer.size());if(in.gcount()&&EVP_DigestUpdate(ctx,buffer.data(),in.gcount())!=1)throw std::runtime_error("OpenSSL SHA update failed");}
 unsigned char digest[EVP_MAX_MD_SIZE];unsigned int n=0;if(EVP_DigestFinal_ex(ctx,digest,&n)!=1)throw std::runtime_error("OpenSSL SHA final failed");EVP_MD_CTX_free(ctx);
 std::ostringstream s;s<<std::hex<<std::setfill('0');for(unsigned int i=0;i<n;i++)s<<std::setw(2)<<(int)digest[i];return s.str();
#endif
}
struct Context {
 std::string objPath,rawPath;Eigen::MatrixXd V,FN,VN,EN;Eigen::MatrixXi F,E;Eigen::VectorXi EMAP;igl::AABB<Eigen::MatrixXd,3> tree;std::once_flag attachFlag;Grid*attachedGrid=nullptr;const void*attachedModel=nullptr;
 Context(){
  objPath=required("CAMPAIGN_HYBRID_OBJ");rawPath=required("CAMPAIGN_HYBRID_RAW");
  if(!std::filesystem::path(objPath).is_absolute()||!std::filesystem::path(rawPath).is_absolute())throw std::runtime_error("Hybrid paths must be absolute");
  if(sha256(objPath)!=lower(required("CAMPAIGN_HYBRID_OBJ_SHA256")))throw std::runtime_error("Hybrid OBJ SHA256 mismatch");
  if(sha256(rawPath)!=lower(required("CAMPAIGN_HYBRID_RAW_SHA256")))throw std::runtime_error("Hybrid RAW SHA256 mismatch");
  if(!igl::readOBJ(objPath,V,F)||F.cols()!=3||F.rows()!=std::stoll(required("CAMPAIGN_HYBRID_TRIANGLES")))throw std::runtime_error("Hybrid triangle count mismatch");
  tree.init(V,F);igl::per_face_normals(V,F,FN);
  igl::per_vertex_normals(V,F,igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_ANGLE,FN,VN);
  igl::per_edge_normals(V,F,igl::PER_EDGE_NORMALS_WEIGHTING_TYPE_UNIFORM,FN,EN,E,EMAP);
 }
 void attach(Grid*grid,const void*model,const std::string&path){
  std::call_once(attachFlag,[&]{
   if(!std::filesystem::equivalent(path,rawPath))throw std::runtime_error("Hybrid model RAW differs from contracted RAW");
   const Eigen::RowVector3d lo=V.colwise().minCoeff();const Eigen::RowVector3d hi=V.colwise().maxCoeff();for(int k=0;k<3;k++)if(grid->getPMin()[k]>lo[k]+1e-10||grid->getPMax()[k]<hi[k]-1e-10)throw std::runtime_error("Hybrid grid box does not contain source mesh");
   attachedGrid=grid;attachedModel=model;
  });
  if(attachedGrid!=grid||attachedModel!=model)throw std::runtime_error("Hybrid contract permits one static identity model/grid per fresh process only");
 }
};
std::once_flag contextFlag;std::unique_ptr<Context>context;
Context&get(){std::call_once(contextFlag,[]{context=std::make_unique<Context>();});return *context;}
ExactContact exactGeometry(Context&c,const Eigen::RowVector3d&a){
 Eigen::MatrixXd P(1,3),C,N;Eigen::VectorXd S;Eigen::VectorXi I;P.row(0)=a;
 igl::signed_distance_pseudonormal(P,c.V,c.F,c.tree,c.FN,c.VN,c.EN,c.EMAP,S,I,C,N);
 ExactContact out;double du=(P.row(0)-C.row(0)).norm();
 for(int k=0;k<3;k++){out.point[k]=C(0,k);out.normal[k]=du>1e-12?(P(0,k)-C(0,k))*(S(0)<0?-1:1)/du:N(0,k);}
 out.signedDistance=S(0);return out;
}
bool query(Grid*grid,const void*model,const std::string&rawPath,const sofa::type::Vec3&p,double radius,double pw,double ps,ExactContact&out){
 ++calls;auto&c=get();c.attach(grid,model,rawPath);
 if(std::abs(pw-.0002)>1e-12||std::abs(ps)>1e-12)throw std::runtime_error("Hybrid requires wall proximity=.0002 and sphere proximity=0");
 const double gate=radius+.002; // exactly old LMD alarm (2mm), distinct from contact offset
 if(grid->inGrid(p)){
  const auto&h=grid->getCellWidth();double bound=.5*std::sqrt(h[0]*h[0]+h[1]*h[1]+h[2]*h[2])+1e-9;
  if(grid->interp(p)>gate+bound){++coarseRejected;return false;}
 }else ++outsideGrid;
 ++exactEvaluations;out=exactGeometry(c,{p[0],p[1],p[2]});return out.signedDistance<gate;
}
} // namespace
extern "C" ADAPTER_EXPORT int campaignHybridPrepare(){try{campaign_hybrid::get();return 1;}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 0;}}
extern "C" ADAPTER_EXPORT const char*campaignHybridStats(){using namespace campaign_hybrid;static thread_local std::string s;std::ostringstream o;o<<"{\"calls\":"<<calls.load()<<",\"coarseRejected\":"<<coarseRejected.load()<<",\"exactEvaluations\":"<<exactEvaluations.load()<<",\"outsideGrid\":"<<outsideGrid.load()<<"}";s=o.str();return s.c_str();}
extern "C" ADAPTER_EXPORT int campaignHybridExactQuery(const double*p,double*out){try{using namespace campaign_hybrid;auto a=exactGeometry(get(),{p[0],p[1],p[2]});out[0]=a.signedDistance;for(int k=0;k<3;k++){out[k+1]=a.point[k];out[k+4]=a.normal[k];}return 1;}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 0;}}

