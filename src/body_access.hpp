#pragma once

class JoltSpace3D;

class JoltBodyAccessRead3D {
public:
	JoltBodyAccessRead3D(const JoltSpace3D& p_space, const JPH::BodyID& p_jid, bool p_lock);

	bool is_valid() const { return lock.Succeeded(); }

	const JPH::Body& get_body() const { return lock.GetBody(); }

private:
	JPH::BodyLockRead lock;
};

class JoltBodyAccessWrite3D {
public:
	JoltBodyAccessWrite3D(const JoltSpace3D& p_space, const JPH::BodyID& p_jid, bool p_lock);

	bool is_valid() const { return lock.Succeeded(); }

	JPH::Body& get_body() const { return lock.GetBody(); }

private:
	JPH::BodyLockWrite lock;
};
