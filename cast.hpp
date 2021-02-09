#pragma once
#include <OGRE/OgreVector.h>
#include <bullet/LinearMath/btVector3.h>
#include <bullet/LinearMath/btQuaternion.h>

inline btVector3 to_bullet(Ogre::Vector3 const & v)
{
	return btVector3{v.x, v.y, v.z};
}

inline Ogre::Vector3 to_ogre(btVector3 const & v)
{
	return Ogre::Vector3{v.x(), v.y(), v.z()};
}

inline Ogre::Quaternion to_ogre(btQuaternion const & q)
{
	return Ogre::Quaternion{q.w(), q.x(), q.y(), q.z()};
}
