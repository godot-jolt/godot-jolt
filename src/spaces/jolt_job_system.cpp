#include "jolt_job_system.hpp"

#include "servers/jolt_project_settings.hpp"

JoltJobSystem::JoltJobSystem()
	: JPH::JobSystemWithBarrier(JPH::cMaxPhysicsBarriers)
	, jobs(JPH::cMaxPhysicsJobs) {
	singleton = this;

	const int32_t max_threads = JoltProjectSettings::get_max_threads();

	if (max_threads != -1) {
		thread_count = max_threads;
	} else {
		thread_count = OS::get_singleton()->get_processor_count();
	}
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

void JoltJobSystem::Job::push_completed(Job* p_job) {
	Job* prev_head = nullptr;

	do {
		prev_head = completed_head;
		p_job->completed_next = prev_head;
	} while (!completed_head.compare_exchange_weak(prev_head, p_job));
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

void JoltJobSystem::Job::queue() {
	AddRef();

	// HACK(mihe): Ideally we would use Jolt's actual job name here, but I'd rather not incur the
	// overhead of a memory allocation or thread-safe lookup every time we create/queue a task. So
	// instead we use the same cached description for all of them.
	static const String description("JoltPhysics");

	task_id = WorkerThreadPool::get_singleton()->add_native_task(&execute, this, true, description);
}

void JoltJobSystem::Job::execute(void* p_user_data) {
	auto* job = static_cast<Job*>(p_user_data);

	job->Execute();
	job->Release();
}

int JoltJobSystem::GetMaxConcurrency() const {
	return thread_count;
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
			"Godot Jolt's job system exceeded maximum number of jobs. This should not happen. "
			"Waiting for jobs to become available."
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
	Job::push_completed(static_cast<Job*>(p_job));
}
