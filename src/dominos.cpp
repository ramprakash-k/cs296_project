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


#include "cs296_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
#include <math.h>
using namespace std;

#include "dominos.hpp"

double pi = 3.14159265359;

namespace cs296
{
  //!  This constructor defines all the bodies present in the simulation.
  
	dominos_t::dominos_t()
	{
	    //! \b Ground: \n
	    //! Variable: b1 (b2Body*):
	    //! It is a pointer to the body ground 
	    
	    {
			b2Body* b1;
			b2EdgeShape shape1; 	//! Variable: shape (b2EdgeShape): Shape of the ground (180 length line)
			shape1.Set(b2Vec2(-90,0), b2Vec2(90,0));
			b2FixtureDef f;
			f.shape=(&shape1);
			f.density=10;
			f.friction = 0;
			f.restitution = 0;
			b2BodyDef bd; 		//! Variable: bd (b2BodyDef): Body properties (default)
			b1 = m_world->CreateBody(&bd);
			b1->CreateFixture(&f);
		}
	    
	    b2Body* b1;
		b2EdgeShape shape1; 	//! Variable: shape (b2EdgeShape): Shape of the ground (180 length line)
		shape1.Set(b2Vec2(0,0), b2Vec2(0,0));
		b2BodyDef bd; 		//! Variable: bd (b2BodyDef): Body properties (default)
		bd.position.Set(0,22.5);
		//~ bd.type=b2_dynamicBody;
		b1 = m_world->CreateBody(&bd);
		b1->CreateFixture(&shape1, 0.0f);
	    
		b2Body* box1;
		b2BodyDef *bd1 = new b2BodyDef;	//! Variable: bd (b2BodyDef*): Body properties (dynamic, position -10,15, fixed rotation)
		bd1->type = b2_dynamicBody;
		bd1->fixedRotation=false;
		bd1->position.Set(0,22.5);
		b2FixtureDef f;
		f.density=1;
		f.friction = 0;
		f.restitution = 1;
		box1 = m_world->CreateBody(bd1);
		b2EdgeShape shape;
		
		b2Body* spherebody;
		b2CircleShape circle;	//! Variable: circle (b2CircleShape): Shape of the spheres (0.5 radius circles)
		circle.m_radius = 0.7;	
		b2FixtureDef ballfd;	//! Variable: ballfd (b2FixtureDef): Fixture of the spheres (density 1, friction 0, restitution 0)
		ballfd.shape = &circle;
		ballfd.density = 20.0f;
		ballfd.friction = 0.0f;
		ballfd.restitution = 1.0f;
		b2BodyDef ballbd;
		ballbd.type = b2_dynamicBody;
		b2Vec2 a[4];
		b2PolygonShape ss;
		for(int i=0;i<20;i++)
		{
			int j=i+1;
			a[0].Set(20.f*sin(i*pi/10),20.f*cos(i*pi/10));
			a[1].Set(20.f*sin(j*pi/10),20.f*cos(j*pi/10));
			a[2].Set(19.9f*sin(j*pi/10),19.9f*cos(j*pi/10));
			a[3].Set(19.9f*sin(i*pi/10),19.9f*cos(i*pi/10));
			ss.Set(a,4);f.shape=(&ss);box1->CreateFixture(&f);
			a[0].Set(16.f*sin(i*pi/10),16.f*cos(i*pi/10));
			a[1].Set(16.f*sin(j*pi/10),16.f*cos(j*pi/10));
			a[2].Set(15.9f*sin(j*pi/10),15.9f*cos(j*pi/10));
			a[3].Set(15.9f*sin(i*pi/10),15.9f*cos(i*pi/10));
			ss.Set(a,4);f.shape=(&ss);box1->CreateFixture(&f);
			shape.Set(b2Vec2(16.f*sin(i*pi/10),16.f*cos(i*pi/10)),b2Vec2(20.f*sin(j*pi/10),20.f*cos(j*pi/10)));
			f.shape=(&shape);
			box1->CreateFixture(&f);
			ballbd.position.Set(18.f*sin(i*pi/10),22.5 + 18.f*cos(i*pi/10));
			spherebody = m_world->CreateBody(&ballbd);
			spherebody->CreateFixture(&ballfd);
		}
		
		b2RevoluteJointDef jd;
		b2Vec2 anchor;	//! Variable: anchor (b2Vec2): position -37,40
		anchor.Set(0.f, 22.5f);
		jd.Initialize(b1, box1, anchor);
		m_world->CreateJoint(&jd);
		box1->SetAngularVelocity(-pi/3);
		

		b2Body* gear[6];
		float clock_center_x=0.0f,clock_center_y=20.0f;
		float 	gear_center_x[]={20.0,18.0,18.0,0.0,0.0,-17.0},
				gear_center_y[]={-5.0,17.0,17.0,0.0,0.0,7.0};
		float p=0.5,d=1.0;
		float gear_angle[]={0.0,0.0,0.0,0.0,0.0,5.0};
		int gear_teeth[]={9,72,24,72,18,72};
		//[ 9 tooth motor gear,
		//  72 tooth minute, 
		//  24 tooth secondory, 
		//  72 tooth intermediate, 
		//  18 tooth secondary, 
		//  72 tooth hour]
		 enum _entityCategory {
			g0	=	0x0001,
			g1	=	0x0002,
			g2	=	0x0004,
			g3	=	0x0008,
			g4	=	0x0010,
			g5	=	0x0020,
		  };
		for(int i=0;i<6;i++){
			if(i==0){
				f.filter.categoryBits	=	g0;
				f.filter.maskBits		=	g1;
				}
			if(i==1){
				f.filter.categoryBits	=	g1;
				f.filter.maskBits		=	g0;
				}
			if(i==2){
				f.filter.categoryBits	=	g2;
				f.filter.maskBits		=	g3;
				}
			if(i==3){
				f.filter.categoryBits	=	g3;
				f.filter.maskBits		=	g2;
				}
			if(i==4){
				f.filter.categoryBits	=	g4;
				f.filter.maskBits		=	g5;
				}
			if(i==5){
				f.filter.categoryBits	=	g5;
				f.filter.maskBits		=	g4;
				}
			int t=2*gear_teeth[i];
			float r0=p/(2*sin(pi/t));
			float r1=r0+d/2,r2=r0+d;
			float temp_angle=gear_angle[i];

			bd1->position.Set(gear_center_x[i]+clock_center_x,gear_center_y[i]+clock_center_y);
			gear[i] = m_world->CreateBody(bd1);
			b2CircleShape cirle;
			circle.m_p.Set(0.0,0.0);
			circle.m_radius=r0;
			f.shape=(&circle);
			gear[i]->CreateFixture(&f);

			b2Vec2 toothShape[6];
			float theta=pi/gear_teeth[i];
			for(int j=0;j<gear_teeth[i];j++){
				toothShape[0]=b2Vec2(r0*cos(temp_angle),r0*sin(temp_angle));
				toothShape[1]=b2Vec2(r1*cos(temp_angle),r1*sin(temp_angle));
				toothShape[2]=b2Vec2(r2*cos(temp_angle+theta/3),r2*sin(temp_angle+theta/3));
				toothShape[3]=b2Vec2(r2*cos(temp_angle+2*theta/3),r2*sin(temp_angle+2*theta/3));
				toothShape[4]=b2Vec2(r1*cos(temp_angle+theta),r1*sin(temp_angle+theta));
				toothShape[5]=b2Vec2(r0*cos(temp_angle+theta),r0*sin(temp_angle+theta));
				ss.Set(toothShape,6);
				f.shape=(&ss);
				f.filter.groupIndex=gear_index[i];
				gear[i]->CreateFixture(&f);
				temp_angle+=4*pi/t;
			}
			
			anchor.Set(gear_center_x[i]+clock_center_x,gear_center_y[i]+clock_center_y);
			jd.Initialize(b1,gear[i],anchor);
			m_world->CreateJoint(&jd);
			gear[i]->SetAngularVelocity(-pi/3);
		}
	}


  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
