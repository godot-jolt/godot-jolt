#include "jolt_job_system.hpp"

namespace {

constexpr int32_t GDJOLT_MAX_BARRIERS = 8;
constexpr int32_t GDJOLT_MAX_JOBS = 2048;

} // namespace

JoltJobSystem::JoltJobSystem()
	: JPH::JobSystemWithBarrier(GDJOLT_MAX_BARRIERS)
	, jobs(GDJOLT_MAX_JOBS) {
	singleton = this;
}

void JoltJobSystem::pre_step() {
	// Nothing to do
}

void JoltJobSystem::post_step() {
	while (Job* job = Job::pop_completed()) {
		jobs.destruct(job);
	}
}

JoltJobSystem::Job::Job(
	const char* p_name,
	JPH::ColorArg p_color,
	JPH::JobSystem* p_job_system,
	const JPH::JobSystem::JobFunction& p_job_function,
	JPH::uint32 p_dependency_count
)
	: JPH::JobSystem::Job(p_name, p_color, p_job_system, p_job_function, p_dependency_count) { }

JoltJobSystem::Job::~Job() {
	if (task_id != -1) {
		WorkerThreadPool::get_singleton()->wait_for_task_completion(task_id);
	}
}

void JoltJobSystem::Job::queue() {
	AddRef();

	// HACK(mihe): Ideally we would use Jolt's actual job name here, but I'd rather not incur the
	// overhead of a memory allocation or thread-safe lookup every time we create/queue a task. So
	// instead we use the same cached description for all of them.
	static const String description("JoltPhysics3D");

	task_id = WorkerThreadPool::get_singleton()->add_native_task(&execute, this, true, description);
}

void JoltJobSystem::Job::push_completed() {
	Job* prev_head = nullptr;

	do {
		prev_head = completed_head;
		completed_next = prev_head;
	} while (!completed_head.compare_exchange_weak(prev_head, this));
}

JoltJobSystem::Job* JoltJobSystem::Job::pop_completed() {
	Job* prev_head = nullptr;

	do {
		prev_head = completed_head;

		if (prev_head == nullptr) {
			return nullptr;
		}
	} while (!completed_head.compare_exchange_weak(prev_head, prev_head->completed_next));

	return prev_head;
}

void JoltJobSystem::Job::execute(void* p_user_data) {
	auto* job = static_cast<Job*>(p_user_data);

	job->Execute();
	job->Release();
}

int JoltJobSystem::GetMaxConcurrency() const {
	// HACK(mihe): Ideally we would use `WorkerThreadPool::get_thread_count` here, but that method
	// is unfortunately not exposed in the bindings.
	return OS::get_singleton()->get_processor_count();
}

JPH::JobHandle JoltJobSystem::CreateJob(
	const char* p_name,
	JPH::ColorArg p_color,
	const JPH::JobSystem::JobFunction& p_job_function,
	JPH::uint32 p_dependency_count
) {
	Job* job = nullptr;

	while (true) {
		job = jobs.construct(p_name, p_color, this, p_job_function, p_dependency_count);

		if (job != nullptr) {
			break;
		}

		WARN_PRINT_ONCE(
			"Job system exceed maximum number of jobs. "
			"Waiting for jobs to become available. "
			"Consider increasing maximum number of jobs."
		);

		OS::get_singleton()->delay_usec(100);
	}

	// This will increment the job's reference count, so must happen before we queue the job
	JPH::JobHandle job_handle(job);

	if (p_dependency_count == 0) {
		QueueJob(job);
	}

	return job_handle;
}

void JoltJobSystem::QueueJob(JPH::JobSystem::Job* p_job) {
	static_cast<Job*>(p_job)->queue();
}

void JoltJobSystem::QueueJobs(JPH::JobSystem::Job** p_jobs, JPH::uint p_job_count) {
	for (JPH::uint i = 0; i < p_job_count; ++i) {
		QueueJob(p_jobs[i]);
	}
}

void JoltJobSystem::FreeJob(JPH::JobSystem::Job* p_job) {
	static_cast<Job*>(p_job)->push_completed();
}
