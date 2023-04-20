#pragma once

class JoltJobSystem final : public JPH::JobSystemWithBarrier {
public:
	JoltJobSystem();

	static JoltJobSystem* get_singleton() { return singleton; }

	void pre_step();

	void post_step();

private:
	class Job : public JPH::JobSystem::Job {
	public:
		Job(const char* p_name,
			JPH::ColorArg p_color,
			JPH::JobSystem* p_job_system,
			const JPH::JobSystem::JobFunction& p_job_function,
			JPH::uint32 p_dependency_count);

		Job(const Job& p_other) = delete;

		Job(Job&& p_other) = delete;

		~Job();

		static void push_completed(Job* p_job);

		static Job* pop_completed();

		void queue();

		Job& operator=(const Job& p_other) = delete;

		Job& operator=(Job&& p_other) = delete;

	private:
		static void execute(void* p_user_data);

		inline static std::atomic<Job*> completed_head = nullptr;

		int64_t task_id = -1;

		std::atomic<Job*> completed_next = nullptr;
	};

	int GetMaxConcurrency() const override;

	JPH::JobHandle CreateJob(
		const char* p_name,
		JPH::ColorArg p_color,
		const JPH::JobSystem::JobFunction& p_job_function,
		JPH::uint32 p_dependency_count = 0
	) override;

	void QueueJob(JPH::JobSystem::Job* p_job) override;

	void QueueJobs(JPH::JobSystem::Job** p_jobs, JPH::uint p_job_count) override;

	void FreeJob(JPH::JobSystem::Job* p_job) override;

	inline static JoltJobSystem* singleton = nullptr;

	FreeList<Job> jobs;

	int32_t thread_count = 0;
};
