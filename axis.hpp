#pragma once
#include <Ogre.h>

//! axis object implementation from [A simple Axis Object for debug purposes](http://wiki.ogre3d.org/ManualObject+AxisObject) article
class AxisObject
{
	enum BoxParts
	{
		BOX_NONE = 0x00,
		BOX_TOP = 0x01,
		BOX_BOT = 0x02,
		BOX_FRONT = 0x04,
		BOX_BACK = 0x08,
		BOX_LEFT = 0x10,
		BOX_RIGHT = 0x20,
		BOX_ALL = 0xFF
	};

public:
	Ogre::ManualObject * createAxis(Ogre::SceneManager * scene, Ogre::String const & name, Ogre::Real scale);

private:
	void addMaterial(Ogre::String const & mat, Ogre::ColourValue const & clr, Ogre::SceneBlendType sbt);
	void addBox(Ogre::ManualObject * obj, Ogre::Vector3 dim, Ogre::Vector3 pos, Ogre::ColourValue color, short boxMask);
};
