#include <utility>
#include <ostream>
#include "physics.hpp"

using std::move, std::make_pair, std::swap;
using std::size;
using std::ostream;

namespace physics {

btVector3 calculate_local_inertia(btCollisionShape & shape, btScalar mass);

body::body(shape_type && shape, btTransform && T, btScalar mass)
	: _shape{move(shape)}
	, _motion{move(T)}
	, _body{mass, &_motion, _shape.get(), calculate_local_inertia(*_shape, mass)}
{
	assert(_shape && "shape required");
}

btVector3 const & body::position() const
{
	return _body.getWorldTransform().getOrigin();
}

btVector3 calculate_local_inertia(btCollisionShape & shape, btScalar mass)
{
	btVector3 local_inertia = {0, 0, 0};
	if (mass > 0)
		shape.calculateLocalInertia(mass, local_inertia);
	return local_inertia;
}


world::world()
	: _dispatcher{&_config}
	, _world{&_dispatcher, &_pair_cache, &_solver, &_config}
{}

void world::add_body(body * b)
{
	_world.addRigidBody(&b->rigid_body());
}

void world::remove_body(body * b)
{
	_world.removeRigidBody(&b->rigid_body());
}

void world::simulate(btScalar time_step, int sub_steps)
{
	_world.stepSimulation(time_step, sub_steps);
	handle_collisions();
}

world::collision_range world::collision_objects()
{
	btCollisionObjectArray const & colls = _world.getCollisionObjectArray();
	return collision_range{&colls[0], &colls[size(colls)]};
}

void world::subscribe_collisions(collision_listener * l)
{
	_collision_listeners.push_back(l);
}

void world::unsubscribe_collisions(collision_listener * l)
{
	auto it = find(begin(_collision_listeners), end(_collision_listeners), l);
	if (it != end(_collision_listeners))
		_collision_listeners.erase(it);
}

void world::handle_collisions()
{
	// collisions this update
	collision_pairs pairs_this_update;
	for (int i = 0; i < _dispatcher.getNumManifolds(); ++i)
	{
		btPersistentManifold * manifold = _dispatcher.getManifoldByIndexInternal(i);
		if (manifold->getNumContacts() > 0)
		{
			auto body0 = manifold->getBody0();
			auto body1 = manifold->getBody1();

			// always create the pair in a predictable order
			bool const swapped = body0 > body1;
			auto sorted_body_a = swapped ? body1 : body0;
			auto sorted_body_b = swapped ? body0 : body1;
			auto collision = make_pair(sorted_body_a, sorted_body_b);
			pairs_this_update.insert(collision);

			if (_last_collisions.find(collision) == _last_collisions.end())
				collision_event((btCollisionObject *)body0, (btCollisionObject *)body1);
		}
	}

	// collisions removed this update
	collision_pairs removed_pairs;
	set_difference(_last_collisions.begin(), _last_collisions.end(),
		pairs_this_update.begin(), pairs_this_update.end(), inserter(removed_pairs, removed_pairs.begin()));

	for (auto const & collision : removed_pairs)
		separation_event((btCollisionObject *)collision.first, (btCollisionObject *)collision.second);

	swap(_last_collisions, pairs_this_update);
}

void world::collision_event(btCollisionObject * a, btCollisionObject * b)
{
	for (auto * l : _collision_listeners)
		l->on_collision(a, b);
}

void world::separation_event(btCollisionObject * a, btCollisionObject * b)
{
	for (auto * l : _collision_listeners)
		l->on_separation(a, b);
}

btTransform translate(btVector3 const & v)
{
	btTransform T;  // uninitialized by default
	T.setIdentity();
	T.setOrigin(v);
	return T;
}

}  // physics

ostream & operator<<(ostream & o, btVector3 const & v)
{
	o << "(" << v.getX() << ", " << v.getY() << ", " << v.getZ() << ")";
	return o;
}
