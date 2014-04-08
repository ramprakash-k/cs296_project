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
	    
	    float xb1=0.0,yb1=22.5;
	    
	    b2Body* b1;
		b2EdgeShape shape1; 	//! Variable: shape (b2EdgeShape): Shape of the ground (180 length line)
		shape1.Set(b2Vec2(0,0), b2Vec2(0,0));
		b2BodyDef bd; 		//! Variable: bd (b2BodyDef): Body properties (default)
		bd.position.Set(xb1,yb1);
		//~ bd.type=b2_dynamicBody;
		b1 = m_world->CreateBody(&bd);
		b1->CreateFixture(&shape1, 0.0f);
	    
		b2Body* box1;
		b2BodyDef *bd1 = new b2BodyDef;	//! Variable: bd (b2BodyDef*): Body properties (dynamic, position -10,15, fixed rotation)
		//~ bd1->type = b2_dynamicBody;
		bd1->fixedRotation=false;
		bd1->position.Set(xb1,yb1);
		b2FixtureDef f;
		f.density=1;
		f.friction = 0;
		f.restitution = 0;
		box1 = m_world->CreateBody(bd1);
		b2EdgeShape shape;
		
		b2Body* spherebody;
		b2CircleShape circle;	//! Variable: circle (b2CircleShape): Shape of the spheres (1 radius circles)
		circle.m_radius = 1;	
		b2FixtureDef ballfd;	//! Variable: ballfd (b2FixtureDef): Fixture of the spheres (density 100, friction 0, restitution 0)
		ballfd.shape = &circle;
		ballfd.density = 100.0f;
		ballfd.friction = 0.0f;
		ballfd.restitution = 0.0f;
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
			a[0].Set(12.f*sin(i*pi/10),12.f*cos(i*pi/10));
			a[1].Set(12.f*sin(j*pi/10),12.f*cos(j*pi/10));
			a[2].Set(11.9f*sin(j*pi/10),11.9f*cos(j*pi/10));
			a[3].Set(11.9f*sin(i*pi/10),11.9f*cos(i*pi/10));
			ss.Set(a,4);f.shape=(&ss);box1->CreateFixture(&f);
			shape.Set(b2Vec2(12.f*sin(i*pi/10),12.f*cos(i*pi/10)),b2Vec2(20.f*sin(j*pi/10),20.f*cos(j*pi/10)));
			f.shape=(&shape);
			box1->CreateFixture(&f);
		}
		
		ballbd.position.Set(xb1+-2,yb1 + 12.7);
		spherebody = m_world->CreateBody(&ballbd);
		spherebody->CreateFixture(&ballfd);
		
		ballbd.position.Set(xb1+3.3,yb1 + 12.4);
		spherebody = m_world->CreateBody(&ballbd);
		spherebody->CreateFixture(&ballfd);
		
		ballbd.position.Set(xb1+6.8,yb1 + 10.8);
		spherebody = m_world->CreateBody(&ballbd);
		spherebody->CreateFixture(&ballfd);
		
		ballbd.position.Set(xb1+10,yb1 + 8);
		spherebody = m_world->CreateBody(&ballbd);
		spherebody->CreateFixture(&ballfd);
		
		ballbd.position.Set(xb1+18.4,yb1 + 1.8);
		spherebody = m_world->CreateBody(&ballbd);
		spherebody->CreateFixture(&ballfd);
		
		ballbd.position.Set(xb1+18.3,yb1 - 4.2);
		spherebody = m_world->CreateBody(&ballbd);
		spherebody->CreateFixture(&ballfd);
		
		ballbd.position.Set(xb1+16.2,yb1 - 9.5);
		spherebody = m_world->CreateBody(&ballbd);
		spherebody->CreateFixture(&ballfd);
		
		ballbd.position.Set(xb1+12.5,yb1 - 14);
		spherebody = m_world->CreateBody(&ballbd);
		spherebody->CreateFixture(&ballfd);
		
		ballbd.position.Set(xb1+7.3,yb1 - 17.2);
		spherebody = m_world->CreateBody(&ballbd);
		spherebody->CreateFixture(&ballfd);
		
		ballbd.position.Set(xb1+1.7,yb1 - 18.6);
		spherebody = m_world->CreateBody(&ballbd);
		spherebody->CreateFixture(&ballfd);
		
		ballbd.position.Set(xb1+-0.6,yb1 - 18.7);
		spherebody = m_world->CreateBody(&ballbd);
		spherebody->CreateFixture(&ballfd);
		
		ballbd.position.Set(xb1+-6.3,yb1 - 17.6);
		spherebody = m_world->CreateBody(&ballbd);
		spherebody->CreateFixture(&ballfd);
		
		ballbd.position.Set(xb1+-11.6,yb1 - 14.9);
		spherebody = m_world->CreateBody(&ballbd);
		spherebody->CreateFixture(&ballfd);
		
		ballbd.position.Set(xb1+-15.7,yb1 - 10.6);
		spherebody = m_world->CreateBody(&ballbd);
		spherebody->CreateFixture(&ballfd);
		
		ballbd.position.Set(xb1+-11.6,yb1 - 5.9);
		spherebody = m_world->CreateBody(&ballbd);
		spherebody->CreateFixture(&ballfd);
		
		ballbd.position.Set(xb1+-12.7,yb1 - 2);
		spherebody = m_world->CreateBody(&ballbd);
		spherebody->CreateFixture(&ballfd);
		
		ballbd.position.Set(xb1+-12.7,yb1 + 2.1);
		spherebody = m_world->CreateBody(&ballbd);
		spherebody->CreateFixture(&ballfd);
		
		ballbd.position.Set(xb1+-11.4,yb1 + 5.8);
		spherebody = m_world->CreateBody(&ballbd);
		spherebody->CreateFixture(&ballfd);
		
		ballbd.position.Set(xb1+-9.1,yb1 + 9.2);
		spherebody = m_world->CreateBody(&ballbd);
		spherebody->CreateFixture(&ballfd);
		
		ballbd.position.Set(xb1+-5.7,yb1 + 11.3);
		spherebody = m_world->CreateBody(&ballbd);
		spherebody->CreateFixture(&ballfd);
		
		b2RevoluteJointDef jd;
		b2Vec2 anchor;	//! Variable: anchor (b2Vec2): position -37,40
		anchor.Set(xb1,yb1);
		jd.Initialize(b1, box1, anchor);
		m_world->CreateJoint(&jd);
		box1->SetAngularVelocity(-pi/3);
		

		b2Body* gear[6];
		float clock_center_x=-30.0f,clock_center_y=20.0f;
		float 	gear_center_x[]={28,15.2,15.2,0.0,0.0,-14.1},
				gear_center_y[]={-4.5,7.0,7.0,0.0,0.0,7.0};
		float p=0.5,d=0.9;
		float gear_angle[]={0.0,0.0,0.0,0.0,0.0,5.0};
		int gear_teeth[]={10,72,24,72,18,72};
		b2RevoluteJoint* rev_joint_gear[6];
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
			float r0=p*t/(2*pi);
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
				gear[i]->CreateFixture(&f);
				temp_angle+=4*pi/t;
			}
			
			anchor.Set(gear_center_x[i]+clock_center_x,gear_center_y[i]+clock_center_y);
			jd.Initialize(b1,gear[i],anchor);
			rev_joint_gear[i] = (b2RevoluteJoint*)m_world->CreateJoint(&jd);
		}
		gear[5]->SetAngularVelocity(2*pi);
		
		b2GearJointDef grj;
		grj.bodyA = gear[3];
		grj.bodyB = gear[4];
		grj.joint1 = rev_joint_gear[3];
		grj.joint2 = rev_joint_gear[4];
		grj.ratio = -1;
		m_world->CreateJoint(&grj);
		
		grj.bodyA = gear[1];
		grj.bodyB = gear[2];
		grj.joint1 = rev_joint_gear[1];
		grj.joint2 = rev_joint_gear[2];
		grj.ratio = -1;
		m_world->CreateJoint(&grj);
	}





  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
