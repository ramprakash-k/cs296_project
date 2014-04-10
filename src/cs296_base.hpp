/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/* 
 * Project for CS 296 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * Group 1 : Siddharth Patel , Ramprakash K , Viplov Jain
 */


#ifndef _CS296BASE_HPP_
#define _CS296BASE_HPP_

#include "render.hpp"
#include <Box2D/Box2D.h>
#include <cstdlib>

#define	RAND_LIMIT 32767

namespace cs296
{

  //! What is the difference between a class and a struct in C++?
  class base_sim_t;
  struct settings_t;
  
  //! Why do we use a typedef
  typedef base_sim_t* sim_create_fcn(); 

  //! Simulation settings. Some can be controlled in the GUI.
  struct settings_t
  {
    /*!This struct stores the whole information on the simulation of the world.
     What to be shown and what not to be shown.The speed of the simulation ,etc.
     It only has data.*/
    
    //! Notice the initialization of the class members in the constructor
    //! How is this happening?
    settings_t() :
      view_center(0.0f, 20.0f),         //! Sets the center of the screen at (0.0f,20.0f)
      hz(60.0f),                        //! Sets the number of simulation steps per second at 60 per frame.
      velocity_iterations(8),           //! Sets the number of velocity iterations at 8.
      position_iterations(3),           //! Sets the number of position iterations at 3.
      draw_shapes(1),                   //! Enables the drawing of shapes.
      draw_joints(1),                   //! Enables the drawing of joints.
      draw_AABBs(0),                    //! Disables the drawing of Axis Aligned Bounding Boxes(AABB) of the bodies.
      draw_pairs(0),                    //! Disables drawing of pairs.
      draw_contact_points(0),           //! Disables the drawing of contact points.
      draw_contact_normals(0),          //! Disables the drawing of normals at the points of contact.
      draw_contact_forces(0),           //! Disables the drawing of contact forces.
      draw_friction_forces(0),          //! Disables the drawing of forces of friction.
      draw_COMs(0),                     //! Disables the drawing of Centre of Masses of the objects.
      draw_stats(0),                    //! Disables showing stats.
      draw_profile(0),                  //! Disables showing profile.
      enable_warm_starting(1),          //! Enables warm starting which caches the distances.
      enable_continuous(1),             //! Enables continuous collision.
      enable_sub_stepping(0),           //! Enables sub stepping.
      pause(0),                         //! Disables the paused state,i.e. it is not paused in the start.
      single_step(0)                    //! Disables single step mode.
    {}
    
    b2Vec2 view_center;                 
	//!< This variable sets the center of the screen.
	//! It can be changed using arrow keys.
    float32 hz;                          
	//!< This variables sets the number of simulation steps per second.
	//! It can be changed in the Sim steps per frame box.
    int32 velocity_iterations;
	//!< This variables sets the number velocity iterations.
	//! It can be changed in the velocity iterations box.
    int32 position_iterations;
	//!< This variables sets the number of position iterations.
	//! It can be changed in the position iterations box.
    int32 draw_shapes;
	//!< This variables describes whether to draw shapes or not.
	//! It can be toggled in the shapes check-box.
    int32 draw_joints;
	//!< This variables describes whether to draw joints between the bodies or not.
	//! It can be toggled in the joints check-box.
    int32 draw_AABBs;
	//!< This variables describes whether to draw Axis Aligned Bounding Boxes(AABB) of the bodies or not.
	//! It can be toggled in the AABB check-box.
    int32 draw_pairs;
	//!< This variables describes whether to draw pairs or not.
	//! It cant be toggled from GUI.
    int32 draw_contact_points;
	//!< This variables describes whether to draw contact points or not.
	//! It cant be toggled from GUI.
    int32 draw_contact_normals;
	//!< This variables describes whether to draw normal at the point of contact or not.
	//! It cant be toggled from GUI.
    int32 draw_contact_forces;
	//!< This variables describes whether to draw forces at the contact point or not.
	//! It cant be toggled from GUI.
    int32 draw_friction_forces;
	//!< This variables describes whether to draw friction forces or not.
	//! It cant be toggled from GUI.
    int32 draw_COMs;
	//!< This variables describes whether to draw centre of masses or not.
	//! It cant be toggled from GUI.
    int32 draw_stats;
	//!< This variables describes whether to show stats or not.
	//! It can be toggled in the stats check-box.
    int32 draw_profile;
	//!< This variables describes whether to show profile or not.
	//! It can be toggled in the profile check-box.
    int32 enable_warm_starting;
	//!< This variables describes whether to enable warm starting(catching pairwise distances of the bodies) or not.
	//! It can be toggled in the Warm Starting check-box.
    int32 enable_continuous;
	//!< This variables describes whether to enable continuous collisions or not.
	//! Continuous collisions prevents tunneling.
    int32 enable_sub_stepping;
	//!< This variables describes whether to enable sub stepping or not.
	//! It can be toggled in the Sub-Stepping check-box.
    int32 pause;
	//!< This variables describes whether to pause the simulation profile or not.
	//! It can be toggled in the pause button.
    int32 single_step;
	//!< This variables describes whether to run the simulation stepwise or not.
	//! It can be used by pressing the Single Step button.
  };
  
  struct sim_t
  /*!This class stores the whole world of the simulation.
     It stores a name for the world and a base_sim_t variable
     which has all the information of the world of the simulation.
     It has both data and a function.*/
  {
    
    
    const char *name;                   //!<This const char* variable stores the name of the world.
    sim_create_fcn *create_fcn;         //!<This sim_create_fcn variable stores the world of the simulation itself.

    sim_t(const char *_name, sim_create_fcn *_create_fcn):   
      name(_name), create_fcn(_create_fcn) {;}//!<This is the constructor function for sim_t.
    
  };
  
  extern sim_t *sim;
  
  
  const int32 k_max_contact_points = 2048;  
  struct contact_point_t   //!This struct defines a contact point and stores its properties.It only has data.
  {
    
    
    b2Fixture* fixtureA;
    //!<It is the fixture of the first body.
    b2Fixture* fixtureB;
    //!<It is the fixture of the second body.
    b2Vec2 normal;
    //!<It stores the normal vector at the point of contact.
    b2Vec2 position;
    //!<It stores the position vector of the point of contact.
    b2PointState state;
    //!<It stores the state of the point of contact.
  };
  
  class base_sim_t : public b2ContactListener    
  /*!This class stores the objects of the simulation and takes care of the calculations for collisions and movement.
     It has both data and functions.*/
  {

  public:
    
    base_sim_t();
    //!<Starts the simulation.
    
    
    // Virtual destructors - amazing objects. Why are these necessary?
    virtual ~base_sim_t();
    //!<Stops the simulation.
    
    void set_text_line(int32 line) { m_text_line = line; } //!<Prints a message.
    
    
    void draw_title(int x, int y, const char *string);
    //!<Writes the title "string" at coordinates (x,y).
    //!The string is "Dominos" and position is top-left by default.
    
    virtual void step(settings_t* settings);                            //!<Sets the simulation settings according to the variable settings.

    virtual void keyboard(unsigned char key) { B2_NOT_USED(key); }     //!<Used to assign a function to a keyboard key.
    virtual void keyboard_up(unsigned char key) { B2_NOT_USED(key); }  //!<Used to assign a function to the keyboard arrow keys.
    
    void shift_mouse_down(const b2Vec2& p) { B2_NOT_USED(p); }         //!<Used to assign a function to shift mouse down operation.
    virtual void mouse_down(const b2Vec2& p) { B2_NOT_USED(p); }       //!<Used to assign a function to mouse down operation.
    virtual void mouse_up(const b2Vec2& p) { B2_NOT_USED(p); }         //!<Used to assign a function to mouse up operation.
    void mouse_move(const b2Vec2& p) { B2_NOT_USED(p); }               //!<Used to assign a function to the movement of the mouse.
    
    
    virtual void joint_destroyed(b2Joint* joint) { B2_NOT_USED(joint); } //!< Let derived tests know that a joint was destroyed.
    
    
    // Callbacks for derived classes.
    
    virtual void begin_contact(b2Contact* contact) { B2_NOT_USED(contact); }    //!<Used to assign a function on the begin of a contact.e.g. create a sound.
    virtual void end_contact(b2Contact* contact) { B2_NOT_USED(contact); }      //!<Used to assign a function on the end of a contact.
    virtual void pre_solve(b2Contact* contact, const b2Manifold* oldManifold);  //!<This makes the calculations for the next step of the simulation and makes the new manifold from the old one.
    virtual void post_solve(const b2Contact* contact, const b2ContactImpulse* impulse)
    {
      B2_NOT_USED(contact);
      B2_NOT_USED(impulse);
    }                                                                           //!<This will be called after the calculations for the manifold are made.

  //!How are protected members different from private memebers of a class in C++ ?
  protected:

    //! What are Friend classes?
    friend class contact_listener_t;
    
    b2Body* m_ground_body;                             //!<This b2Body* variable is the pointer to the groung body.
    b2AABB m_world_AABB;                               //!<This b2AABB variable stores the Axis Aligned Bounding Box(AABB) of the whole world of the Simulation.
    contact_point_t m_points[k_max_contact_points];    //!<This is the array of the contact_point_t variabls and stores the contact point details of all the contact points in the simulation.
    int32 m_point_count;                               //!<This int32 variable stores the number of contact points.

    debug_draw_t m_debug_draw;                         //!<This debug_draw_t variable stores a shape.
    int32 m_text_line;                                 //!<This int32 variable represents a text.
    b2World* m_world;                                  //!<This b2World* variable is the pointer to the whole simulation world.

    int32 m_step_count;                                //!<This int32 variable stores the step count of the simulation.
    
    b2Profile m_max_profile;                           //!<This b2Profile variable stores the maximum values of the physical variables in the simulation.
    b2Profile m_total_profile;                         //!<This b2Profile variable stores the total values of the physical variables in the simulation.
  };
}

#endif
