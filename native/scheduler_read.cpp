#include <sofa/simulation/MainTaskSchedulerRegistry.h>
#include <sofa/simulation/TaskScheduler.h>
extern "C" __attribute__((visibility("default"))) int campaign_last_scheduler_thread_count() {
    const auto& entry=sofa::simulation::MainTaskSchedulerRegistry::getLastInserted();
    if (!entry || !entry->second) return -1;
    return static_cast<int>(entry->second->getThreadCount());
}
extern "C" __attribute__((visibility("default"))) int campaign_named_scheduler_thread_count(const char* name) {
    if (!name) return -1;
    auto* scheduler=sofa::simulation::MainTaskSchedulerRegistry::getTaskScheduler(name);
    return scheduler ? static_cast<int>(scheduler->getThreadCount()) : -1;
}
