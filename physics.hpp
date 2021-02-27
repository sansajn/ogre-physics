#pragma once
#include <set>
#include <memory>
#include <iosfwd>
#include <boost/range/iterator_range.hpp>
#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletCollision/btBulletCollisionCommon.h>

namespace physics {

class body
{
public:
	/*! represents body shape
	To create box shape, type
	\code
	shape_type box = make_unique<btBoxShape>(btVector3{50, 50, 50});
	\endcode */
	using shape_type = std::unique_ptr<btCollisionShape>;

	body(shape_type && shape, btTransform && T, btScalar mass = 0);
	btVector3 const & position() const;

	// native geters
	btRigidBody & rigid_body() {return _body;}

	template <typename T>
	bool is_same(T * p) const {return (void *)p == &_body;}

private:
	shape_type _shape;
	btDefaultMotionState _motion;
	btRigidBody _body;
};

struct collision_listener
{
	virtual void on_collision(btCollisionObject * a, btCollisionObject * b) {}
	virtual void on_separation(btCollisionObject * a, btCollisionObject * b) {}
};

class world
{
public:
	using collision_range = boost::iterator_range<btCollisionObject * const *>;

	world();
	void add_body(body * b);
	void remove_body(body * b);
	void simulate(btScalar time_step, int sub_steps = 10);

	collision_range collision_objects();

	void subscribe_collisions(collision_listener * l);
	void unsubscribe_collisions(collision_listener * l);

	btDiscreteDynamicsWorld & native() {return _world;}

private:
	using collision_pairs = std::set<std::pair<btCollisionObject const *,
		btCollisionObject const *>>;

	void handle_collisions();
	void collision_event(btCollisionObject * a, btCollisionObject * b);
	void separation_event(btCollisionObject * a, btCollisionObject * b);

	btDefaultCollisionConfiguration _config;
	btCollisionDispatcher _dispatcher;
	btDbvtBroadphase _pair_cache;
	btSequentialImpulseConstraintSolver _solver;
	btDiscreteDynamicsWorld _world;

	collision_pairs _last_collisions;
	std::vector<collision_listener *> _collision_listeners;
};

// helpers
btTransform translate(btVector3 const & v);

} // physics

// STL support
std::ostream & operator<<(std::ostream & o, btVector3 const & v);
