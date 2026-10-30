#include <sofa/simulation/MainTaskSchedulerFactory.h>
#include <sofa/simulation/TaskScheduler.h>
extern "C" __attribute__((visibility("default"))) int campaign_configure_default_scheduler(unsigned int count) {
    if (!count || count>64) return -2;
    auto* scheduler=sofa::simulation::MainTaskSchedulerFactory::createInRegistry();
    if (!scheduler) return -1;
    scheduler->init(count);
    return static_cast<int>(scheduler->getThreadCount());
}
