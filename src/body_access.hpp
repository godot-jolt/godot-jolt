#pragma once

class JoltPhysicsSpace3D;

class BodyAccessRead {
public:
	BodyAccessRead(const JoltPhysicsSpace3D& p_space, const JPH::BodyID& p_jid, bool p_lock);

	bool is_valid() const { return lock.Succeeded(); }

	const JPH::Body& get_body() const { return lock.GetBody(); }

private:
	JPH::BodyLockRead lock;
};

class BodyAccessWrite {
public:
	BodyAccessWrite(const JoltPhysicsSpace3D& p_space, const JPH::BodyID& p_jid, bool p_lock);

	bool is_valid() const { return lock.Succeeded(); }

	JPH::Body& get_body() const { return lock.GetBody(); }

private:
	JPH::BodyLockWrite lock;
};
