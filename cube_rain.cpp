// physics in cube rain scence
#include <vector>
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

using std::vector;
using std::string, std::to_string;
using std::unique_ptr, std::make_unique;
using std::random_device, std::default_random_engine;
using std::cout, std::endl;
using std::chrono::steady_clock, std::chrono::duration;

using Ogre::SceneManager, // Ogre::vector name collision with std::vector so `using namespace Ogre` cannot be used
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

cube_object new_cube();


class ogre_app
	: public ApplicationContext, public InputListener, public RenderTargetListener
{
public:
	ogre_app();
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
	SceneNode * create_cube_node(SceneManager & scene, cube_object const & cube);

	unique_ptr<CameraMan> _cameraman;
	vector<cube_object> _cubes;  // cube pool
	vector<SceneNode *> _cube_nodes;
	InputListenerChain _input_listeners;
	unique_ptr<ImGuiInputListener> _imgui_listener;
	SceneManager * _scene = nullptr;

	// settings
	int _cube_count = 300;
};

namespace std {

string to_string(CameraStyle style);

}  // std

void ogre_app::update(duration<double> dt)
{
	// handle number of cubes option (if changed)
	int prev_cube_count = size(_cubes);
	_cubes.resize(_cube_count);

	SceneNode & root = *_scene->getRootSceneNode();

	if (_cube_count < prev_cube_count)  // remove additional cube nodes from scene graph
	{
		for_each(begin(_cube_nodes) + _cube_count, end(_cube_nodes),
			[&root](SceneNode * nd){root.removeChild(nd);});

		_cube_nodes.resize(_cube_count);
	}
	else if (_cube_count > prev_cube_count)  // add additional cubes to scene graph
	{
		_cubes.resize(_cube_count);
		_cube_nodes.resize(_cube_count);

		for (int i = 0; i < _cube_count - prev_cube_count; ++i)
		{
			cube_object & cube = _cubes[prev_cube_count + i];
			cube = new_cube();

			_cube_nodes[prev_cube_count + i] = create_cube_node(*_scene, cube);
		}
	}

	// update cubes
	constexpr Real fall_speed = 3;
	constexpr Real fall_off_threshold = -10.0;

	auto cube_node_it = begin(_cube_nodes);
	for (cube_object & cube : _cubes)
	{
		cube.position.y -= fall_speed * (2.0 - cube.scale) * dt.count();
		if (cube.position.y < fall_off_threshold)
			cube = new_cube();

		(*cube_node_it)->setPosition(cube.position);  // update scene position
		++cube_node_it;
	}
}

void ogre_app::setup_scene(SceneManager & scene)
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

	auto cube_nodes_it = begin(_cube_nodes);

	// add cubes to scene
	for (cube_object & cube : _cubes)
	{
		// save node for later update
		*cube_nodes_it = create_cube_node(scene, cube);
		++cube_nodes_it;
	}

	// axis
	AxisObject axis;
	Ogre::ManualObject * axis_model = axis.createAxis(&scene, "axis", 0.5);
	SceneNode * axis_nd = root_nd->createChildSceneNode();
	axis_nd->attachObject(axis_model);
}

void ogre_app::update_gui()
{
	ImGui::Begin("Info");  // begin window

	ImGui::SliderInt("Number of cubes", &_cube_count, 100, 1500);

	ImGui::End();  // end window

	ImGui::Render();
}

void ogre_app::setup()
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

void ogre_app::go()
{
	initApp();

	if (getRoot()->getRenderSystem())
		getRoot()->startRendering();  // rendering loop

	closeApp();
}

ogre_app::ogre_app()
	: ApplicationContext{"ogre cuberain"}
{
	// initialize cubes
	_cubes.resize(_cube_count);
	for (cube_object & cube : _cubes)
		cube = new_cube();

	_cube_nodes.resize(_cube_count);
}

bool ogre_app::keyPressed(KeyboardEvent const & evt)
{
	if (evt.keysym.sym == SDLK_ESCAPE)
	{
		getRoot()->queueEndRendering();
		return true;
	}
	else
		_input_listeners.keyPressed(evt);

	return true;
}

bool ogre_app::keyReleased(KeyboardEvent const & evt)
{
	return _input_listeners.keyReleased(evt);
}

bool ogre_app::mouseMoved(MouseMotionEvent const & evt)
{
	return _input_listeners.mouseMoved(evt);
}

bool ogre_app::mousePressed(MouseButtonEvent const & evt)
{
	return _input_listeners.mousePressed(evt);
}

bool ogre_app::mouseReleased(MouseButtonEvent const & evt)
{
	return _input_listeners.mouseReleased(evt);
}

void ogre_app::frameRendered(Ogre::FrameEvent const & evt)
{
	_cameraman->frameRendered(evt);
}

bool ogre_app::textInput(TextInputEvent const & evt)
{
	return _input_listeners.textInput(evt);
}

bool ogre_app::frameStarted(FrameEvent const & evt)
{
	duration<double> dt{evt.timeSinceLastFrame};
	update(dt);
	return ApplicationContext::frameStarted(evt);
}

void ogre_app::preViewportUpdate(RenderTargetViewportEvent const & evt)
{
	if (!evt.source->getOverlaysEnabled())
		return;

	ImGuiOverlay::NewFrame();

	update_gui();
}

SceneNode * ogre_app::create_cube_node(SceneManager & scene, cube_object const & cube)
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


cube_object new_cube()
{
	static random_device rd;
	static default_random_engine rand{rd()};

	return cube_object{
		Vector3{
			(rand() % 15) - 7.f,
			7.f + (rand() % 30),
			(rand() % 15) - 7.f},
		0.7f + ((rand() % 70)/100.f)  // scale between 0.7 and 0.7+0.7
	};
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
	ogre_app app;
	app.go();
	return 0;
}
