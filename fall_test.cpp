// physics in cube rain scence
#include <vector>
#include <set>
#include <map>
#include <utility>
#include <string>
#include <memory>
#include <random>
#include <chrono>
#include <iostream>
#include <Ogre.h>
#include <OgreApplicationContext.h>
#include <OgreCameraMan.h>
#include <OgreTrays.h>
#include <OgreImGuiOverlay.h>
#include <OgreImGuiInputListener.h>
#include "axis.hpp"
#include "physics.hpp"
#include "cast.hpp"

using std::vector, std::set, std::map;
using std::pair;
using std::string, std::to_string;
using std::unique_ptr, std::make_unique;
using std::random_device, std::default_random_engine;
using std::cout, std::endl;
using std::chrono::steady_clock, std::chrono::duration;

using namespace std::chrono_literals;

using Ogre::SceneManager,  // Ogre::vector name collision with std::vector so `using namespace Ogre` cannot be used there
	Ogre::SceneNode,
	Ogre::FrameListener,
	Ogre::FrameEvent,
	Ogre::Light,
	Ogre::Camera,
	Ogre::Entity,
	Ogre::ColourValue,
	Ogre::Vector3,
	Ogre::Real,
	Ogre::Node,
	Ogre::OverlayManager,
	Ogre::ImGuiOverlay,
	Ogre::RenderTargetListener,
	Ogre::RenderTargetViewportEvent;

using namespace OgreBites;


Vector3 const camera_position = {0, 0, 10};

// flyweight pattern
struct cube_object
{
	Vector3 position;
	Real scale;  // value between 0.7 and 1.4 used to scale cube model
};

// helpers
cube_object new_cube();
btTransform translate(Vector3 const & v);

struct collision_record
{
	steady_clock::time_point t_impact;
};


class cube_rain
	: public ApplicationContext, public InputListener, public RenderTargetListener
{
public:
	cube_rain();
	void go();  //!< app entry point

private:
	void setup_scene(SceneManager & scene);
	void update(duration<double> dt);
	void update_gui();

	// ApplicationContext overrides
	void setup() override;
	bool frameStarted(FrameEvent const & evt) override;

	// InputListener events
	bool keyPressed(KeyboardEvent const & evt) override;
	bool keyReleased(KeyboardEvent const & evt) override;
	bool mouseMoved(MouseMotionEvent const & evt) override;
	bool mousePressed(MouseButtonEvent const & evt) override;
	bool mouseReleased(MouseButtonEvent const & evt) override;
	void frameRendered(Ogre::FrameEvent const & evt) override;
	bool textInput(TextInputEvent const & evt) override;

	// RenderTargetListener events
	void preViewportUpdate(RenderTargetViewportEvent const & evt) override;

	// helpers
	void add_cubes(size_t n);
	void remove_cubes(size_t n);
	SceneNode * create_cube_node(SceneManager & scene, cube_object const & cube);
	physics::body * create_cube_body(cube_object const & cube, SceneNode * nd);

	unique_ptr<CameraMan> _cameraman;
	vector<cube_object> _cubes;  // cube pool
	vector<SceneNode *> _cube_nodes;
	map<SceneNode *, collision_record> _highlighted_cube_nodes;
	InputListenerChain _input_listeners;
	unique_ptr<ImGuiInputListener> _imgui_listener;
	SceneManager * _scene = nullptr;

	// settings
	int _cube_count = 60;
	double _time_dilation = 1.0;

	// physics related stuff ...
	physics::world _world;
	vector<physics::body *> _cube_bodies;  // btRigidBody is not default constructible, that is why *
};

namespace std {

string to_string(CameraStyle style);

}  // std


struct collision_collector : public physics::collision_listener
{
	set<btCollisionObject *> result;

	void clear()
	{
		result.clear();
	}

	void on_collision(btCollisionObject * a, btCollisionObject * b) override
	{
		result.insert({a, b});
	}
};

void cube_rain::update(duration<double> dt)
{
	// handle number of cubes option (if changed)
	int prev_cube_count = size(_cubes);

	if (_cube_count < prev_cube_count)
		remove_cubes(prev_cube_count - _cube_count);
	else if (_cube_count > prev_cube_count)
		add_cubes(_cube_count - prev_cube_count);

	collision_collector collisions;
	_world.subscribe_collisions(&collisions);

	_world.simulate(dt.count());

	// highlight all collided cubes
	steady_clock::time_point now = steady_clock::now();

	// find out all collided cubes and highlight them
	for (btCollisionObject * o : collisions.result)
	{
		SceneNode * nd = static_cast<SceneNode *>(o->getUserPointer());
		_highlighted_cube_nodes.insert_or_assign(nd, collision_record{now});
		dynamic_cast<Entity *>(nd->getAttachedObject(0))->setMaterialName("cube_collision_color");
	}

	// removes old highlights (after 1s)
	vector<map<SceneNode *, collision_record>::const_iterator> old_highlights;
	for (auto it = cbegin(_highlighted_cube_nodes); it != cend(_highlighted_cube_nodes); ++it)
	{
		if (now - it->second.t_impact > 250ms)
		{
			old_highlights.push_back(it);
			dynamic_cast<Entity *>(it->first->getAttachedObject(0))->setMaterialName("cube_color");
		}
	}

	for (auto it : old_highlights)
		_highlighted_cube_nodes.erase(it);


	_world.unsubscribe_collisions(&collisions);

	// update cubes (position, rotations)
	assert(size(_cubes) == size(_cube_nodes) && size(_cubes) == size(_cube_bodies));

	constexpr Real fall_off_threshold = -10.0;

	auto cube_node_it = begin(_cube_nodes);
	auto cube_body_it = begin(_cube_bodies);
	for (cube_object & cube : _cubes)
	{
		if (cube.position.y > fall_off_threshold)
		{
			cube.position = to_ogre((*cube_body_it)->position());
		}
		else  // reuse cubes too far from start position
		{
			cube = new_cube();
			(*cube_body_it)->rigid_body().setWorldTransform(translate(cube.position));
		}

		btQuaternion orientation = (*cube_body_it)->rigid_body().getOrientation();
		++cube_body_it;

		(*cube_node_it)->setPosition(cube.position);  // update cube position
		(*cube_node_it)->setOrientation(to_ogre(orientation));
		++cube_node_it;
	}
}

void cube_rain::setup_scene(SceneManager & scene)
{
	SceneNode * root_nd = scene.getRootSceneNode();

	// without light we would just get a black screen
	scene.setAmbientLight(ColourValue{0.5, 0.5, 0.5});
	SceneNode * light_nd = root_nd->createChildSceneNode();
	Light * light = scene.createLight("light");
	light_nd->setPosition(20, 80, 50);
	light_nd->attachObject(light);

	// create camera so we can observe scene
	SceneNode * camera_nd = root_nd->createChildSceneNode();
	camera_nd->setPosition(camera_position);
	camera_nd->lookAt(Vector3{0, 0, -1}, Node::TS_PARENT);

	Camera * camera = scene.createCamera("main_camera");
	camera->setNearClipDistance(0.1);  // specific to this sample
	camera->setAutoAspectRatio(true);
	camera_nd->attachObject(camera);

	_cameraman = make_unique<CameraMan>(camera_nd);
	_cameraman->setStyle(CS_ORBIT);
	cout << "camera style: " << to_string(_cameraman->getStyle()) << endl;

	getRenderWindow()->addViewport(camera);  // render into the main window

	add_cubes(_cube_count);

	// axis
	AxisObject axis;
	Ogre::ManualObject * axis_model = axis.createAxis(&scene, "axis", 0.5);
	SceneNode * axis_nd = root_nd->createChildSceneNode();
	axis_nd->attachObject(axis_model);
}

void cube_rain::update_gui()
{
	ImGui::Begin("Info");  // begin window

	ImGui::SliderInt("Number of cubes", &_cube_count, 100, 1500);

	ImGui::End();  // end window

	ImGui::Render();
}

void cube_rain::setup()
{
	ApplicationContext::setup();
	addInputListener(this);  // register for input events

	_scene = getRoot()->createSceneManager();

	// setup imgui overlay
	{
		ImGuiOverlay * imgui = new ImGuiOverlay();
		imgui->setZOrder(300);
		imgui->show();
		OverlayManager::getSingleton().addOverlay(imgui);  // imgui is now owned by OverlayManager
	}

	_scene->addRenderQueueListener(getOverlaySystem());

	getRenderWindow()->addListener(this);  // register for render targets events

	// register our scene with the RTSS
	Ogre::RTShader::ShaderGenerator * shadergen = Ogre::RTShader::ShaderGenerator::getSingletonPtr();
	shadergen->addSceneManager(_scene);

	setup_scene(*_scene);

	// input listeners
	_imgui_listener = make_unique<ImGuiInputListener>();
	_input_listeners = InputListenerChain({_imgui_listener.get(), _cameraman.get()});
}

void cube_rain::go()
{
	initApp();

	if (getRoot()->getRenderSystem())
		getRoot()->startRendering();  // rendering loop

	closeApp();
}

cube_rain::cube_rain()
	: ApplicationContext{"ogre cuberain"}
{
	_world.native().setGravity(btVector3{0,0,0});  // turn off gravity
}

bool cube_rain::keyPressed(KeyboardEvent const & evt)
{
	if (evt.keysym.sym == SDLK_ESCAPE)
	{
		getRoot()->queueEndRendering();
		return true;
	}
	else if (evt.keysym.sym == SDLK_SPACE)
	{
		if (_time_dilation == 0.0)  // comparison is safe, not any math operations
			_time_dilation = 1.0;
		else
			_time_dilation = 0.0;
	}
	else
		_input_listeners.keyPressed(evt);

	return true;
}

bool cube_rain::keyReleased(KeyboardEvent const & evt)
{
	return _input_listeners.keyReleased(evt);
}

bool cube_rain::mouseMoved(MouseMotionEvent const & evt)
{
	return _input_listeners.mouseMoved(evt);
}

bool cube_rain::mousePressed(MouseButtonEvent const & evt)
{
	return _input_listeners.mousePressed(evt);
}

bool cube_rain::mouseReleased(MouseButtonEvent const & evt)
{
	return _input_listeners.mouseReleased(evt);
}

void cube_rain::frameRendered(Ogre::FrameEvent const & evt)
{
	_cameraman->frameRendered(evt);
}

bool cube_rain::textInput(TextInputEvent const & evt)
{
	return _input_listeners.textInput(evt);
}

bool cube_rain::frameStarted(FrameEvent const & evt)
{
	duration<double> dt{evt.timeSinceLastFrame * _time_dilation};
	update(dt);
	return ApplicationContext::frameStarted(evt);
}

void cube_rain::preViewportUpdate(RenderTargetViewportEvent const & evt)
{
	if (!evt.source->getOverlaysEnabled())
		return;

	ImGuiOverlay::NewFrame();

	update_gui();
}

void cube_rain::add_cubes(size_t n)
{
	size_t prev_cube_count = size(_cubes),
		cube_count = prev_cube_count + n;

	assert(size(_cubes) == size(_cube_nodes) && size(_cubes) == size(_cube_bodies));
	_cubes.resize(cube_count);
	_cube_nodes.resize(cube_count);
	_cube_bodies.resize(cube_count);

	assert(_scene);

	for (size_t i = 0; i < n; ++i)  // for new cubes
	{
		size_t const idx = prev_cube_count + i;

		cube_object & cube = _cubes[idx];
		cube = new_cube();

		SceneNode * nd = create_cube_node(*_scene, cube);
		_cube_nodes[idx] = nd;
		_cube_bodies[idx] = create_cube_body(cube, nd);
	}
}

void cube_rain::remove_cubes(size_t n)
{
	assert(n <= size(_cube_nodes));

	size_t cube_count = size(_cube_nodes) - n;

	SceneNode & root = *_scene->getRootSceneNode();
	for_each(begin(_cube_nodes) + cube_count, end(_cube_nodes),
		[&root](SceneNode * nd){root.removeChild(nd);});
	_cube_nodes.resize(cube_count);

	for_each(begin(_cube_bodies) + cube_count, end(_cube_bodies),
		[this](physics::body * b){
			_world.remove_body(b);
			delete b;
	});
	_cube_bodies.resize(cube_count);

	_cubes.resize(cube_count);

	assert(size(_cubes) == size(_cube_nodes) && size(_cubes) == size(_cube_bodies));
}

SceneNode * cube_rain::create_cube_node(SceneManager & scene, cube_object const & cube)
{
	Entity * cube_model = scene.createEntity(SceneManager::PT_CUBE);
	cube_model->setMaterialName("cube_color");  // see media/cube.material

	SceneNode * nd = scene.getRootSceneNode()->createChildSceneNode(cube.position);
	Real model_scale = 0.2 * (2.0 / cube_model->getBoundingBox().getSize().x);
	Real cube_scale = model_scale * cube.scale;
	nd->setScale(cube_scale, cube_scale, cube_scale);
	nd->attachObject(cube_model);

	return nd;
}

physics::body * cube_rain::create_cube_body(cube_object const & cube, SceneNode * nd)
{
	btScalar mass = 1;
	physics::body * result = new physics::body{
		make_unique<btBoxShape>(btVector3{.5, .5, .5} * cube.scale),
		physics::translate(to_bullet(cube.position)),
		mass
	};

	btScalar const fall_speed = 3 * (2.0 - cube.scale);
	result->rigid_body().setLinearVelocity(btVector3{0, -fall_speed, 0});

	result->rigid_body().setUserPointer(nd);  // link with OGRE

	_world.add_body(result);

	return result;
}

btTransform translate(Vector3 const & v)
{
	btTransform T;
	T.setIdentity();
	T.setOrigin(btVector3{v.x, v.y, v.z});
	return T;
}

cube_object new_cube()
{
	static random_device rd;
	static default_random_engine rand{rd()};

	// generate three grid cube indices for 10x10x10 grid cube
	constexpr unsigned size = 10;
	unsigned i = rand() % size,
		j = rand() % size,
		k = rand() % size;

	float x = (i - 0.5f*size) * (1.4f+1.f),  // from -7 to 7
		y = (j + 7.f) * (1.4f+1.f),
		z = (k - 0.5f*size) * (1.4f+1.f);

	float scale = 0.7f + (rand() % 71)/100.f;  // scale between 0.7 and 0.7+0.7

	return cube_object{Vector3{x, y, z}, scale};
}


namespace std {

string to_string(CameraStyle style)
{
	switch (style)
	{
		case CS_FREELOOK: return "freelook";
		case CS_ORBIT: return "orbit";
		case CS_MANUAL: return "manual";
		default: return "unknown";
	}
}

}  // std

int main(int argc, char * argv[])
{
	cube_rain app;
	app.go();
	return 0;
}
