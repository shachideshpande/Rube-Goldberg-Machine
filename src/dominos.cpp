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
 * Base code for CS 251 Software Systems Lab
 * Department of Computer Science and Engineering, IIT Bombay
 *
 */


#include "cs251_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include "GL/freeglut.h"
#endif

#include <cstring>
 #include <iostream>
using namespace std;

#include "dominos.hpp"

namespace cs251
{
/**  The is the constructor
 * This is the documentation block for the constructor.
 */

dominos_t::dominos_t()
{
    //Ground
    /*! \var b1
      Block for G


Variable : shape Type:b2PolygonShape
This variable is used to define a rectangular shape for a segment component of shape 'G', using SetAsBox() function.

Variable : bd Type:b2BodyDef
This variable is used to define position for a segment component of shape 'G', using position.Set() function.

Variables :g1,g2,g3,g4,stop Type : b2Body*
These variables hold addresses of the actual components of G created by m_world->CreateBody() function. Also, we attach shape and properties
to each of the shape using the variables bd and shape defined above. Body g4 is set to be b2_dynamicBody, since it is a dynamic block which
will slide off from inner stem of G on getting required external impulse. Rest g1,g2,g3 and stop are pointing to various segments that together form
G. The variables bd and shape declared above in this block are reused for each of these segments to define their shapes and fixtures.

Opening Door for G:

Variable :top Type : b2Body*
This variable defines a static circular body upon which the door will be hinged.b2CircleShape cshape and b2BodyDef bd are used as usual to set
circle radius, define position, etc.

Variable: door Type : b2Body*
This variable defines a dynamic rectangular body which will act as door, hinged on the circle 'top'. b2PolygonShape 'shape' and b2BodyDef 'doordef' are
used to set shape and position of the body respectively.

Variable : revoluteJointDef Type : b2RevoluteJointDef
This variable creates a revolute joint between door and top defined above. BodyA is top and BodyB is door. local anchors of both these
bodies is set to (0,0) so that they rotate about their centre. The variable collideConnected is set to false so that both these bodies don't collide
with external bodies.

Motor:

Variable : motorpane Type : b2Body*
This variable is used to create a rectangular dynamic object that will serve as blade of rotating fan.b2PolygonShape type variable 'shape' is used
to define its shape as box of specified height and width, and b2BodyDef type variable bd is used to set position and set the body to be dynamic.

Variable : pin Type b2Body*
This variable creates a static circle which serves as a pin on which the rotating blade will be 'mounted'. b2CircleShape type variable 'cshape' is used to define
radius for it and b2BodyDef type variable bd is used to set position.

Variable : revoluteJointDef Type : b2RevoluteJointDef
This variable creates a revolute joint between motorpane and pin defined above. BodyA is motorpane and BodyB is pin. local anchors of both these
bodies is set to (0,0) so that they rotate about their centre. The variable collideConnected is set to false so that both these bodies don't collide
with external bodies.Now, the enableMotor is set true, and maxMotorTorque and motorSpeed is set for the motor as 1000 MKS and 9 rad/s respectively.
Higher motor torque helps in maintaining motor's speed in in case it is disturbed by any external impulse.

Letter W:
This is composed of 4 segments for W. All the components are static. For each of the 4 segments we have following variables:
Variable : b2 Type : b2Body*
Variable : shape Type : b2PolygonShape
Variable :center Type : const b2Vec2
Variable : resti Type : b2FixtureDef
Each segment of W is further composed of many small rectangular static bodies. Above variables b2, shape and center are used to define the shape,
width, height and position of insertion of each of the components. The restitution is set to 1 using variable resti.These components have been added in for loops, with the position of
insertion defined by the loop variable 'i'. This ensures that the components approximately get added in a parabola-like trajectory.

Also, 2 rectangular plancks have been inserted at the base of 'W' so that balls bouncing over W won't get stuck in the bottom corners of the
letter shape. Again the variables b2,shape,center and resti are used exactly as above for these 2 plancks.

Tunnel to pressure-transferring system:
This is composed of several rectangular static segments. The variables used for each of these rectangular components are as follows:
Variable : shape Type : b2PolygonShape ( Defines box shape with certain width and height, sets inclination of box in radians)
Variable : center Type : const b2Vec2 (Sets point of insertion of component)
Variable : bd Type : b2BodyDef (helpful in defining properties of the body)
Variable : ground Type : b2Body* (Points to the body created in world )
There is a horizontal shelf which holds the ball initially before it follows the tunnel. Now there are 3 inclined static segments that create this
tunnel. 2 vertical static segments create the end part of this tunnel to ultimately deliver the ball to the pressure-transferring system.

Pressure-transferring system ( Food )
This is composed of 2 boxes made of static rectangular bodies, which are connected in between by a small tunnel. These rectangular components have been
defined exactly like the components of Tunnel to pressure-transferring system above. We have used variables b2PolygonShape shape, b2BodyDef bd, const
b2Vec2 center and b2Body* ground exactly like above components of Tunnel to pressure-transferring system. We basically have 2 big boxes holding
the many balls( food for man) with a tunnel between 2 boxes. The box on left side has a funnel which delivers balls (i.e. food) when pressure is applied
on the rectangular platform which settles upon balls in right box. We have a rectangular dynamic plank which serves to cover the balls from above
and acts as a medium to transfer the force impinged by the ball that falls upon it.

Now numerous balls are inserted into the right box. Variables used for this are
Variable : spherebody Type : b2Body* (Points to the ball created in world)
Variable :  ballfd Type : b2FixtureDef ( Defines density, friction and restitution coefficient for the balls)
Variable : circle Type : b2CircleShape (Sets radius for the balls)
Variable : ballbd Type : b2BodyDef ( Sets balls to be of type b2_dynamicBody and sets position of insertion using loop variables i and j)

Using 2 for loops, 198 balls of radius 0.25 are to be inserted into the right box of the system.

Man on a platform, with table and bowl, ready to eat :)
This is again composed of many static rectangular and circle bodies. We use exactly similar variables, i.e. b2PolygonShape shape, b2BodyDef bd,
const b2Vec2 center and b2Body* ground to define the many rectangular components in this system, which includes platform on which the man stands,
table( just a rectangular block), bowl which is composed of 2 slanting and one horizontal rectangular segment, the Man's body and legs. The Man's face is a static circle
shape defined using the usual variables b2CircleShape circle, b2FixtureDef ballfd, b2BodyDef ballbd, and b2Body* spherebody.

S-shape
This has 3 horizontal rectangular static objects, upon which rectangular dominos are kept. Revolute joints are defined between 1 domino on every
horizontal plank with the end of plank.

Top horizontal shelf - b2Body* b2 is defined using using b2PolygonShape, and position coordinates are given using b2BodyDef variable bd.
Domino - b2Body* b4 using b2PolygonShape, and position coordinates are given using b2BodyDef variable bd.
b2RevoluteJointDef jd is used to join the tip of shelf and domino above, using a single absolute anchor.
Exactly similarly ,another b2RevoluteJointDef jd is used to join b2Body* b5 ( 2nd horizontal shelf) and b2Body* b7( 2nd domino at tip of this shelf).
Also, 3rd horizontal shelf is defined by b2Body* b8.

Dominos:
Using for loop, 3 trains of dominos are created on each of the 3 shelves of S above.We use b2PolygonShape, and position coordinates are given using b2BodyDef variable bd.
b2FixtureDef fd is used to define density and shape of dominos.

Box on S:
This is a dynamic rectangular box defined using b2PolygonShape, and position coordinates are given using b2BodyDef variable bd3, fixture defined using
b2FixtureDef* fd3.

Pendulum that knocks of Dominos:
b2Body* b3 and b2Body* b6 are used to define the 2 components of pendulum to be joint by b2RevoluteJointDef jd. b3 and b6 are defined using
using b2PolygonShape, and position coordinates are given using b2BodyDef variable bd. b6 is the square bob of pendulum, and b3 is a 2nd body
used to define a revolute joint with b6, so that the bob b6 can hang on it. The position of bob is set such that it begins to move at the
beginning of simulation.

Block for A Shape:
This is composed of 1 large horizontal shelf making the middle horizontal segment of letter 'A'. There are symmetrically placed shelves on both sides of
this large horizontal shelf, which hold 1 domino each. There are a total of 7 such small horizontal shelves positioned symmetrically
to form letter 'A' together with the large shelf.
Each horizontal shelf block is defined as follows:
Using b2PolygonShape which sets the shape as box of certain large width and small height, sets position and inclination
using shape.SetAsBox() method , and position coordinates are given using b2BodyDef variable bd.
Each domino is defined as follows:
Using b2PolygonShape to set shape of rectangle with small width and certain large height, b2FixtureDef fd to set this shape, and set density and friction
for each domino, and position coordinates are given using b2BodyDef variable bd.

Pulley system:
Variable bd of type b2BodyDef* is declared as dynamic and fixedRotation is set to true.

Parts:
Open box on left:
Here, we have 3 b2FixtureDef* variables fd1,fd2 and fd3. They define density, friction and restitution of the box sides as 10.0, 0.5 and 0 respectively.
3 b2PolygonShape type variables bs1,bs2 and bs3 define the size, orientation and position of the 3 sides of box.Shape of fixtures fd1,fd2 and fd3 is set
to that of bs1,bs2 and bs3. b2Body* type variable box1 is now created in m_world, and fixtures fd1, fd2 and fd3 are added to it.

Open box on right:
Here, we have 3 b2FixtureDef* variables fd4,fd5 and fd6. They define density, friction and restitution of the box sides as 10.0, 0.5 and 0 respectively.
3 b2PolygonShape type variables bs4,bs5 and bs6 define the size, orientation and position of the 3 sides of box.Shape of fixtures fd4,fd5 and fd6 is set
to that of bs4,bs5 and bs6. b2Body* type variable box1 is
now created in m_world, and fixtures fd4, fd5 and fd6 are added to it.

Thus we have 2 big containers which hold balls. Variable myjoint of type b2PulleyJointDef* is used to define a pulley joint with specified centres and ratio as 1.
b2Vec2 is used to define 2 vectors as world anchors on ground. The anchors on 2 boxes are taken to be their world centres, using GetWorldCenter() method.

Pendulum to knock dominos:
This also has revolute joint.

Conveyor Belts:

Parts:
Right motor:
Here, b2BodyDef bd and b2FixtureDef* fd are used to define the fixture and properties of the motor pane that will run and the ends of conveyor
belt. b2PolygonShape type variable shape defines the shape of pane as a box of certain height and width.Similarly, a b2BodyDef bd2 with b2PloygonShape
shape2 is defined as centre of rotation of the motor pane. These 2 bodies are connected using b2RevoluteJointDef type variable jointDef. The motor
function is enabled, maximum torque and motor speed are set to 2 and 3 respectively. Local anchors of both bodies are set to (0,0).

Left Motor:
This has exactly same construction as right motor, only the positional coordinates are different.

Right Circle Around Motor:
This is defined using b2FixtureDef fd. It is imparted circle shape using b2CircleShape 'shape'. Radius and insertion position is set for shape
and the isSensor value in fd is set to true.Density and friction is set for fd as 20 and 0.1 respectively.

Left Circle Around Motor:
This has exactly same construction as right circle, only the positional coordinates are different.

Upper belt ( cardboard)
Thin box shape is defined using b2PolygonShape, and position coordinates are given using b2BodyDef variable bd.

Lower Belt (cardboard)
This has exactly same construction as upper belt, only the positional coordinates are different.

Sensors implementation:( Done in dominos.hpp file)
This is done by defining a b2Fixture* m_sensor in class dominos_t. The BeginContact function is redefined here, i.e. overridden so that when the
box falls on upper belt of conveyor, it gains a horizontal velocity, and does not fall off. This is done by checking the userdata of fixture
coming in contact. The bool value representing whether objects are touching is set to true, and the object falling on conveyor is given velocity by
the SetLinearVelocity() method.


Ball-delivery System:
Here we have 2 trains of spheres defined as usual using b2CircleShape circle, b2BodyDef ballbd and b2FixtureDef ballfd, setting the radiyes, density, restitution and friction.
They are inserted in positions controlled by loop variable i of for loop. A container is defined for the dynamic balls using 3 static rectangular bodies.
They are defined as usual by b2BodyDef bd to set insertion position and b2PolygonShape to define shape and size. A slant rectangular body is also defined
which directs the balls from this container into the container of pulley  system. The slope of the body is defined in bd.Position.Set() method.

Negative gravity balls:
These balls are defined as usual using b2CircleShape shape and b2FixtureDef fd, setting the radiyes, density, restitution and friction.
They are inserted in positions controlled by loop variable i of for loop.Using method SetGravityScale() of b2Body* body variable,
we set the negative scale factor for gravity of balls.

Revolving horizontal platform:
This is a rotating segment made using revolute joint. 2 b2BodyDef type variables bd and bd2 are defined , and respective b2PolygonShape 'shape' and
'shape2' are defined for them. b2FixtureDef* fd is used to define fixture for variable bd. Finally, variable jointDef of type b2RevoluteJointDef is used
to set revolute joint between bd and bd2. The local anchors are set as (0,0) for both bodies and the joint is initialised.


Table :
Table is basically a dynamic horizontal plank defined using b2PolygonShape, and position coordinates are given using b2BodyDef variable bd.Small
static boxes to the left and right are defined in exactly similar way. 2 of them are meant to support the dynamic table from below
and 2 of them stop the table from rising above a certain height when the table is pushed from below.

The legs of table are defined as b2Body* leg1 and b2Body* leg2. They are dynamic rectangular objects defined as usual using b2PolygonShape, and
 position coordinates are given using b2BodyDef variable bd. Now a b2RevuluteJointDef jointDef1 and jointDef2 are used to create
 a revolute joint between leg1 and table, and leg2 and table, respectively. Local anchors are set on both the bodies being joint by
 revolute joints such that the legs hang down from tips of table and the table becomes stable during lift by balls of negative gravity.


     */
    b2Body* b1;
    {

        b2EdgeShape shape;
        shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
        b2BodyDef bd;
        b1 = m_world->CreateBody(&bd);
        b1->CreateFixture(&shape, 0.0f);
    }

m_world->SetContactListener(this);
    //The upper train of small spheres
   {
      b2Body* spherebody;

      b2CircleShape circle;
      circle.m_radius = 0.5;

      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 5.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;

      for (int i = 0; i < 5; ++i)
  {
    b2BodyDef ballbd;
    ballbd.type = b2_dynamicBody;
    ballbd.position.Set(2.0f + i*1.0, 22.0f);
    spherebody = m_world->CreateBody(&ballbd);
    spherebody->CreateFixture(&ballfd);
  }
  }

   //The lower train of small spheres
   {
      b2Body* spherebody;

      b2CircleShape circle;
      circle.m_radius = 0.5;

      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 5.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;

      for (int i = 0; i < 6; ++i)
  {
    b2BodyDef ballbd;
    ballbd.type = b2_dynamicBody;
    ballbd.position.Set(1.5f + i*1.0, 21.3f);
    spherebody = m_world->CreateBody(&ballbd);
    spherebody->CreateFixture(&ballfd);
  }
  }




// left rod of container for keeping balls
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 2.0f);

      b2BodyDef bd;
      bd.position.Set(1.0f, 21.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }


    // right rod of container for keeping balls
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 2.0f);

      b2BodyDef bd;
      bd.position.Set(7.0f, 21.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }

///////////rotating base for keeping balls

   {
    b2Body* b1;
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.4f, 0.4f);

      b2BodyDef bd;
      bd.position.Set(1.0f, 19.0f);
   //   b2Body* ground = m_world->CreateBody(&bd);
     // ground->CreateFixture(&shape, 0.0f);
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 10.0f);

    }

    b2Body* b3;
    {
      b2PolygonShape shape;
      shape.SetAsBox(5.5f, 0.1f);

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 10.0f;
      fd.friction = 0.1f;

      b2BodyDef bd;
    bd.type = b2_dynamicBody;
    bd.position.Set(1.0f , 19.0f);
    b3 = m_world->CreateBody(&bd);
      b3->CreateFixture(&shape, 10.0f);
    }


      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(1.0f, 19.0f);
      jd.Initialize(b1, b3,anchor);


      m_world->CreateJoint(&jd);

  }

// box to keep rod stable
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.5f, 0.5f);

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;
      fd.friction = 0.1f;
    b2BodyDef bd;
    bd.type = b2_dynamicBody;
    bd.position.Set(-1.0f, 19.3f);
    b2Body* body = m_world->CreateBody(&bd);
    body->CreateFixture(&fd);
   // b20 = m_world->CreateBody(&bd);
   // b20->CreateFixture(&fd);
  }


   // left rod to stop ballons
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.2f, 1.1f);

      b2BodyDef bd;
      bd.position.Set(-5.0f, 18.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }

//3rd rod above pulley
    {
        b2Body* b2;
        b2PolygonShape shape;
        b2FixtureDef resti;
        resti.restitution=1.0;
        const b2Vec2 center(8.0,17.0);
        shape.SetAsBox(1.5f, 0.1f,center,0.5f);
        b2BodyDef bd;
        b2 = m_world->CreateBody(&bd);
        b2->CreateFixture(&shape, 10.0f);

    }


    // 2nd rod above pulley
    {
        b2Body* b2;
        b2PolygonShape shape;
        b2FixtureDef resti;
        resti.restitution=1.0;
        const b2Vec2 center(6.0,14.5);
        shape.SetAsBox(1.5f, 0.1f,center,-0.5f);
        b2BodyDef bd;
        b2 = m_world->CreateBody(&bd);
        b2->CreateFixture(&shape, 10.0f);

    }

    //1st rod above pulley
    {
        b2Body* b2;
        b2PolygonShape shape;
        b2FixtureDef resti;
        resti.restitution=1.0;
        const b2Vec2 center(8.0,12.0);
        shape.SetAsBox(2.0f, 0.1f,center,0.5f);
        b2BodyDef bd;
        b2 = m_world->CreateBody(&bd);
        b2->CreateFixture(&shape, 10.0f);

    }





    //The pulley system
   {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-10,15);//////////////////////////////////////////////nooooooooooo
      bd->fixedRotation = true;

      //The open box left
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.1, b2Vec2(16.0f,-15.0f), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 12.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.1,2, b2Vec2(14.0f,-14.0f), 0);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 10.0;
      fd3->friction = 0.5;
      fd3->restitution = 0.f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.1,2, b2Vec2(18.0f,-14.0f), 0);
      fd3->shape = &bs3;

      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);

       //The open box right
      b2FixtureDef *fd4 = new b2FixtureDef;
      fd4->density = 15.0;
      fd4->friction = 0.5;
      fd4->restitution = 0.f;
      fd4->shape = new b2PolygonShape;
      b2PolygonShape bs4;
      bs4.SetAsBox(3.0,0.1, b2Vec2(27.0f,-6.0f), 0);
      fd4->shape = &bs4;
      b2FixtureDef *fd5 = new b2FixtureDef;
      fd5->density = 15.0;
      fd5->friction = 0.5;
      fd5->restitution = 0.f;
      fd5->shape = new b2PolygonShape;
      b2PolygonShape bs5;
      bs5.SetAsBox(0.1,1, b2Vec2(24.0f,-5.0f), 0);
      fd5->shape = &bs5;
      b2FixtureDef *fd6 = new b2FixtureDef;
      fd6->density = 10.0;
      fd6->friction = 0.5;
      fd6->restitution = 0.f;
      fd6->shape = new b2PolygonShape;
      b2PolygonShape bs6;
      bs6.SetAsBox(0.1,1, b2Vec2(30.0f,-5.0f), 0);
      fd6->shape = &bs6;

      b2Body* box2 = m_world->CreateBody(bd);
      box2->CreateFixture(fd4);
      box2->CreateFixture(fd5);
      box2->CreateFixture(fd6);

      // The pulley joint
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(-10, 15); // Anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody2(10, 26); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround1(6.5, 10); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround2(17, 10); // Anchor point for ground 2 in world axis
      float32 ratio = 1.0f; // Define ratio
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);
    }
    {
     b2PolygonShape shape;
      shape.SetAsBox(1.0f, 2.0f);

      b2BodyDef bd;
      bd.position.Set(20.0f, 10.0f);
      bd.type=b2_dynamicBody;
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }

    //negative gravity balls
    {
      b2CircleShape shape;
     // shape.SetAsBox(0.1f, 1.0f);
      shape.m_radius=0.4f;

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;
      fd.friction = 0.1f;

  {
    b2BodyDef bd;
    bd.type = b2_dynamicBody;
    bd.position.Set(-9.5f, 1.0f);
    b2Body* body = m_world->CreateBody(&bd);
    body->SetGravityScale(-0.3);
    body->CreateFixture(&fd);
  }

  {
    b2BodyDef bd;
    bd.type = b2_dynamicBody;
    bd.position.Set(-9.0f, 1.0f);
    b2Body* body = m_world->CreateBody(&bd);
    body->SetGravityScale(-0.3);
    body->CreateFixture(&fd);
  }

  {
    b2BodyDef bd;
    bd.type = b2_dynamicBody;
    bd.position.Set(-10.5f, 1.0f);
    b2Body* body = m_world->CreateBody(&bd);
    body->SetGravityScale(-0.3);
    body->CreateFixture(&fd);
  }

  {
    b2BodyDef bd;
    bd.type = b2_dynamicBody;
    bd.position.Set(-11.0f, 1.0f);
    b2Body* body = m_world->CreateBody(&bd);
    body->SetGravityScale(-0.3);
    body->CreateFixture(&fd);
  }

  {
    b2BodyDef bd;
    bd.type = b2_dynamicBody;
    bd.position.Set(-9.5f, 1.0f);
    b2Body* body = m_world->CreateBody(&bd);
    body->SetGravityScale(-0.3);
    body->CreateFixture(&fd);
  }

  {
    b2BodyDef bd;
    bd.type = b2_dynamicBody;
    bd.position.Set(-9.0f, 1.0f);
    b2Body* body = m_world->CreateBody(&bd);
    body->SetGravityScale(-0.3);
    body->CreateFixture(&fd);
  }

  {
    b2BodyDef bd;
    bd.type = b2_dynamicBody;
    bd.position.Set(-10.5f, 1.0f);
    b2Body* body = m_world->CreateBody(&bd);
    body->SetGravityScale(-0.3);
    body->CreateFixture(&fd);
  }

  {
    b2BodyDef bd;
    bd.type = b2_dynamicBody;
    bd.position.Set(-11.0f, 1.0f);
    b2Body* body = m_world->CreateBody(&bd);
    body->SetGravityScale(-0.3);
    body->CreateFixture(&fd);
  }
    }


    //right box to stop table
    {
      b2PolygonShape shape;
      shape.SetAsBox(2.0f, 0.4f);

      b2BodyDef bd;
      bd.position.Set(-5.0f, 11.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }

    //left box to stop table
    {
      b2PolygonShape shape;
      shape.SetAsBox(2.0f, 0.4f);

      b2BodyDef bd;
      bd.position.Set(-15.0f, 11.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }

    //right box to support table
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 0.4f);

      b2BodyDef bd;
      bd.position.Set(-5.5f, 3.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }

    //left box to support table
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 0.4f);

      b2BodyDef bd;
      bd.position.Set(-13.0f, 3.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }


// table
    {
      b2PolygonShape shape;
      shape.SetAsBox(7.01f, 0.2f);

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 1.0f;
      fd.friction = 0.1f;
    b2BodyDef bd;
    bd.type = b2_dynamicBody;
    bd.position.Set(-10.0f, 3.3f);
    b2Body* body = m_world->CreateBody(&bd);
    body->CreateFixture(&fd);
  }

/////////////////////clamps//////////////////
  //clamp
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 0.1f);

      b2BodyDef bd;
      bd.position.Set(-1.0f, 8.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
    //clamp
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 0.1f);

      b2BodyDef bd;
      bd.position.Set(2.0f, 8.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
//clamp
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 0.1f);

      b2BodyDef bd;
      bd.position.Set(-19.0f, 8.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
      //clamp
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 0.1f);

      b2BodyDef bd;
      bd.position.Set(-22.0f, 8.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }

//////////////////////////////////




///////////left hinge-rod pair involved n formation of table
    {
    b2Body* b2;
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.4f, 0.4f);

      b2BodyDef bd;
      bd.position.Set(-1.0f, 7.0f);
   //   b2Body* ground = m_world->CreateBody(&bd);
     // ground->CreateFixture(&shape, 0.0f);
      b2 = m_world->CreateBody(&bd);
      b2->CreateFixture(&shape, 10.0f);

    }

    b2Body* b4;
    {
      b2PolygonShape shape;
      shape.SetAsBox(3.2f, 0.1f);

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 10.0f;
      fd.friction = 0.1f;

      b2BodyDef bd;
    bd.type = b2_dynamicBody;
    bd.position.Set(-1.0f , 7.0f);
   // b2Body* body = m_world->CreateBody(&bd);
    //body->CreateFixture(&fd);
    b4 = m_world->CreateBody(&bd);
      b4->CreateFixture(&shape, 10.0f);
    }
      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-1.0f, 7.0f);
      jd.Initialize(b2, b4,anchor);

      m_world->CreateJoint(&jd);


  }

///////////right hinge-rod pair involved n formation of table

   {
    b2Body* b1;
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.4f, 0.4f);

      b2BodyDef bd;
      bd.position.Set(-19.0f, 7.0f);
   //   b2Body* ground = m_world->CreateBody(&bd);
     // ground->CreateFixture(&shape, 0.0f);
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 10.0f);

    }

    b2Body* b3;
    {
      b2PolygonShape shape;
      shape.SetAsBox(3.65f, 0.1f);

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 10.0f;
      fd.friction = 0.1f;

      b2BodyDef bd;
    bd.type = b2_dynamicBody;
    bd.position.Set(-19.0f , 7.0f);
    b3 = m_world->CreateBody(&bd);
      b3->CreateFixture(&shape, 10.0f);
    }
      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-19.0f, 7.0f);
      jd.Initialize(b1, b3,anchor);

      m_world->CreateJoint(&jd);
  }

    ////////////////////////////////// S starts ////////////////////////////////////
//Top horizontal shelf in S
    {
        b2Body* b2;
        {
            b2PolygonShape shape;
            shape.SetAsBox(6.0f, 0.25f);

            b2BodyDef bd;
            bd.position.Set(-31.0f, 42.0f);
            //   b2Body* ground = m_world->CreateBody(&bd);
            // ground->CreateFixture(&shape, 0.0f);
            b2 = m_world->CreateBody(&bd);
            b2->CreateFixture(&shape, 10.0f);

        }

        b2Body* b4;
        {
            b2PolygonShape shape;
            shape.SetAsBox(0.1f, 2.0f);

            b2FixtureDef fd;
            fd.shape = &shape;
            fd.density = 20.0f;
            fd.friction = 0.1f;

            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position.Set(-36.5f , 44.25f);
            // b2Body* body = m_world->CreateBody(&bd);
            //body->CreateFixture(&fd);
            b4 = m_world->CreateBody(&bd);
            b4->CreateFixture(&shape, 10.0f);
        }
        b2RevoluteJointDef jd;
        b2Vec2 anchor;
        anchor.Set(-36.5f, 42.0f);
        jd.Initialize(b2, b4,anchor);

        m_world->CreateJoint(&jd);


    }
// second horizontal shelf in S
    {
        b2Body* b5;
        {
            b2PolygonShape shape;
            shape.SetAsBox(6.0f, 0.25f);

            b2BodyDef bd;
            bd.position.Set(-31.0f, 37.3f);
            //   b2Body* ground = m_world->CreateBody(&bd);
            // ground->CreateFixture(&shape, 0.0f);
            b5 = m_world->CreateBody(&bd);
            b5->CreateFixture(&shape, 10.0f);

        }

        b2Body* b7;
        {
            b2PolygonShape shape;
            shape.SetAsBox(0.1f, 2.0f);

            b2FixtureDef fd;
            fd.shape = &shape;
            fd.density = 20.0f;
            fd.friction = 0.1f;

            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position.Set(-25.5f , 39.25f);
            // b2Body* body = m_world->CreateBody(&bd);
            //body->CreateFixture(&fd);
            b7 = m_world->CreateBody(&bd);
            b7->CreateFixture(&shape, 10.0f);
        }
        b2RevoluteJointDef jd;
        b2Vec2 anchor;
        anchor.Set(-25.5f, 37.3f);
        jd.Initialize(b5, b7,anchor);

        m_world->CreateJoint(&jd);


    }

    // third horizontal shelf in S

    b2Body* b8;
    {
        b2PolygonShape shape;
        shape.SetAsBox(6.0f, 0.25f);

        b2BodyDef bd;
        bd.position.Set(-31.0f, 31.6f);
        // b2Body* ground = m_world->CreateBody(&bd);
        // ground->CreateFixture(&shape, 0.0f);
        b8 = m_world->CreateBody(&bd);
        b8->CreateFixture(&shape, 10.0f);

    }



    //Dominos on top horizontal shelf on S
    {
        b2PolygonShape shape;
        shape.SetAsBox(0.1f, 1.0f);

        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 20.0f;
        fd.friction = 0.1f;

        for (int i = 0; i < 10; ++i)
        {
            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position.Set(-35.5f + 1.0f * i, 43.25f);
            b2Body* body = m_world->CreateBody(&bd);
            body->CreateFixture(&fd);
        }
    }

//Dominos on second horizontal shelf of S
    {
        b2PolygonShape shape;
        shape.SetAsBox(0.1f, 1.0f);

        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 20.0f;
        fd.friction = 0.1f;

        for (int i = 0; i < 8; ++i)
        {
            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position.Set(-35.5f + 1.0f * i, 40.25f);
            b2Body* body = m_world->CreateBody(&bd);
            body->CreateFixture(&fd);
        }
    }

    //Dominos on third horizontal shelf of S
    {
        b2PolygonShape shape;
        shape.SetAsBox(0.1f, 1.0f);

        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 20.0f;
        fd.friction = 0.1f;

        for (int i = 2; i < 10; ++i)
        {
            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position.Set(-35.5f + 1.0f * i, 35.25f);
            b2Body* body = m_world->CreateBody(&bd);
            body->CreateFixture(&fd);
        }
    }
    // Box on the S
    {
        b2PolygonShape shape;
        shape.SetAsBox(0.5f, 0.5f);
        b2BodyDef bd3;
        bd3.position.Set(-36.0f, 35.0f);
        bd3.type = b2_dynamicBody;
        b2Body* body = m_world->CreateBody(&bd3);
        b2FixtureDef *fd3 = new b2FixtureDef;
        fd3->density = 5.0f;
        fd3->shape = new b2PolygonShape;
        fd3->shape = &shape;
        m_sensor = body->CreateFixture(fd3);

    }
    //////////////////////////////////S ends ////////////////////////////////////////

    //The pendulum that knocks the dominos off
    {
        b2Body* b3;
        {
            b2PolygonShape shape;
            shape.SetAsBox(0.25f, 0.1f);

            b2BodyDef bd;
            bd.position.Set(-25.5f, 42.0f);
            b3 = m_world->CreateBody(&bd);
            b3->CreateFixture(&shape, 10.0f);
        }

        b2Body* b6;
        {
            b2PolygonShape shape;
            shape.SetAsBox(0.4f, 0.4f);

            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position.Set(-20.5f, 46.0f);
            b6 = m_world->CreateBody(&bd);
            b6->CreateFixture(&shape, 2.0f);
        }

        b2RevoluteJointDef jd;
        b2Vec2 anchor;
        anchor.Set(-25.5f, 55.0f);
        jd.Initialize(b3, b6,anchor);

        m_world->CreateJoint(&jd);
    }

    ////////////////////////////////////////////////////conveyor1 starts /////////////////////////////////////
//right motor
  {
      b2PolygonShape shape;
      shape.SetAsBox(1.2f, 0.2f);

      b2BodyDef bd;
      bd.position.Set(2.0f, 27.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      b2PolygonShape shape2;
      shape2.SetAsBox(0.2f, 2.0f);
      b2BodyDef bd2;
      bd2.position.Set(2.0f, 27.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.enableMotor = true;
      jointDef.motorSpeed = 8;
      jointDef.maxMotorTorque = 2;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
    }
// left motor
    {
      b2PolygonShape shape;
      shape.SetAsBox(1.2f, 0.2f);

      b2BodyDef bd;
      bd.position.Set(-41.0f, 27.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      b2PolygonShape shape2;
      shape2.SetAsBox(0.2f, 2.0f);
      b2BodyDef bd2;
      bd2.position.Set(-41.0f, 27.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.enableMotor = true;
      jointDef.motorSpeed = 8;
      jointDef.maxMotorTorque = 2;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
    }

    //Left circle as seen bu u
    {
      b2CircleShape shape;

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.isSensor = true;
      fd.density = 20.0f;
      fd.friction = 0.1f;
      shape.m_p.Set(-41.0f, 27.0f);
      shape.m_radius = 1.2f;

    b2BodyDef bd;
    b2Body* body = m_world->CreateBody(&bd);
    body->CreateFixture(&fd);
  }

  //Right circle as seen by u

  {
      b2CircleShape shape;

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.isSensor = true;
      fd.density = 20.0f;
      fd.friction = 0.1f;
      shape.m_p.Set(2.0f, 27.0f);
      shape.m_radius = 1.2f;

    b2BodyDef bd;
    b2Body* body = m_world->CreateBody(&bd);
    body->CreateFixture(&fd);
  }

  //horizontal cardboards
  //upper cardboard
  b2Body* b17;
  {
      b2PolygonShape shape;
      shape.SetAsBox(21.5f, 0.01f);

      b2BodyDef bd;
      bd.position.Set(-19.5f, 28.4f);
       b17 = m_world->CreateBody(&bd);
      m_sensor1 = b17->CreateFixture(&shape, 0.0f);
    }
//lower cardboard
    {
      b2PolygonShape shape;
      shape.SetAsBox(21.5f, 0.01f);

      b2BodyDef bd;
      bd.position.Set(-19.5f, 25.6f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);

    }

    /////////////////////////////////////////////////////Conveyor belt ends //////////////////////////////////

    ////////////////////////////////////////////////////box to stop ballons starts/////////////////////////////////////
     {
        b2PolygonShape shape;
        shape.SetAsBox(6.0f, 0.25f);

        b2BodyDef bd;
        bd.position.Set(11.0f, 25.0f);
         b2Body* ground = m_world->CreateBody(&bd);
         ground->CreateFixture(&shape, 0.0f);

    }

{
        b2PolygonShape shape;
        shape.SetAsBox(0.25f, 1.0f);

        b2BodyDef bd;
        bd.position.Set(8.0f, 25.0f);
         b2Body* ground = m_world->CreateBody(&bd);
         ground->CreateFixture(&shape, 0.0f);

    }
    	{

        b2PolygonShape shape;
        shape.SetAsBox(0.25f, 1.0f);

        b2BodyDef bd;
        bd.position.Set(17.0f, 25.0f);
         b2Body* ground = m_world->CreateBody(&bd);
         ground->CreateFixture(&shape, 0.0f);

    }
    ////////////////////////////////////////////////////box to stop ballons ends /////////////////////////////////////

    ///////////////////////// W starts //////////////////////////
    // W 1st seg

for(int i=0; i<7; i++)
    {
        b2Body* b2;
        b2PolygonShape shape;
        b2FixtureDef resti;
        resti.restitution=1.0;
        const b2Vec2 center((-20.0+1.0*i),(40.0-0.2*i*i));
        shape.SetAsBox(1.5f, 0.1f,center,-1.0f);
        b2BodyDef bd;
        b2 = m_world->CreateBody(&bd);
        b2->CreateFixture(&shape, 10.0f);

    }

    //W 4th seg
    for(int i=0; i<6; i++)
    {
        b2Body* b2;
        b2PolygonShape shape;
        b2FixtureDef resti;
        resti.restitution=1.0;
        const b2Vec2 center((-1.0*i),(38.0-0.2*i*i));
        shape.SetAsBox(1.0f, 0.1f,center,1.0f);
        b2BodyDef bd;
        b2 = m_world->CreateBody(&bd);
        b2->CreateFixture(&shape, 10.0f);

    }
// W 3rd seg
    for(int i=0; i<5; i++)
    {
        b2Body* b2;
        b2PolygonShape shape;
        b2FixtureDef resti;
        resti.restitution=1.0;
        const b2Vec2 center((-9.0-1.2*i),(36.0-0.2*i*i));
        shape.SetAsBox(1.0f, 0.1f,center,0.6f);
        b2BodyDef bd;
        b2 = m_world->CreateBody(&bd);
        b2->CreateFixture(&shape, 10.0f);

    }
// W 2nd seg
    for(int i=0; i<5; i++)
    {
        b2Body* b2;
        b2PolygonShape shape;
        b2FixtureDef resti;
        resti.restitution=1.0;
        const b2Vec2 center((-10.0+1.2*i),(36.0-0.2*i*i));
        shape.SetAsBox(0.5+0.1*i, 0.1,center,-0.6f);
        b2BodyDef bd;
        b2 = m_world->CreateBody(&bd);
        b2->CreateFixture(&shape, 10.0f);

    }
    //W plank 1

    {
        b2Body* b2;
        b2PolygonShape shape;
        b2FixtureDef resti;
        resti.restitution=1.0;
        const b2Vec2 center((-14.0),(35.0));
        shape.SetAsBox(1.5, 0.1,center,0.0f);
        b2BodyDef bd;
        b2 = m_world->CreateBody(&bd);
        b2->CreateFixture(&shape, 10.0f);
    }

    //W plank 2

    {
        b2Body* b2;
        b2PolygonShape shape;
        b2FixtureDef resti;
        resti.restitution=1.0;
        const b2Vec2 center((-6.0),(35.0));
        shape.SetAsBox(1.5, 0.1,center,0.0f);
        b2BodyDef bd;
        b2 = m_world->CreateBody(&bd);
        b2->CreateFixture(&shape, 10.0f);
    }

    //  launching plank of ball
    {
        b2Body* b2;
        b2PolygonShape shape;
        b2FixtureDef resti;
        resti.restitution=1.0;
        const b2Vec2 center((-20.0),(41.0));
        shape.SetAsBox(1.0, 0.1,center,-0.2f);
        b2BodyDef bd;
        b2 = m_world->CreateBody(&bd);
        b2->CreateFixture(&shape, 10.0f);
    }

    // The ball
    {
        b2Body* spherebody;

        b2CircleShape circle;
        circle.m_radius = 0.4;

        b2FixtureDef ballfd;
        ballfd.shape = &circle;
        ballfd.density = 8.0f;
        ballfd.friction = 0.0f;
        ballfd.restitution = 1.0f;

        b2BodyDef ballbd;
        ballbd.type = b2_dynamicBody;

        ballbd.position.Set(-20.0f, 45.0f);
        spherebody = m_world->CreateBody(&ballbd);
        spherebody->CreateFixture(&ballfd);

    }

    //Motor
    {
        b2Body* motorpane;
        {
            b2PolygonShape shape;
            b2BodyDef bd;
            shape.SetAsBox(0.2f, 3.5f);
            bd.position.Set(2.0f, 42.5f);
            bd.type = b2_dynamicBody;
            motorpane = m_world->CreateBody(&bd);
            motorpane->CreateFixture(&shape, 0.5f);

        }
        b2Body* pin;
        {
            b2CircleShape cshape;
            b2BodyDef bd;
            cshape.m_radius = 0.4;
            bd.position.Set(2.0f, 42.5f);
            pin = m_world->CreateBody(&bd);
            pin->CreateFixture(&cshape, 0.5f);
        }

        b2RevoluteJointDef revoluteJointDef;
        revoluteJointDef.bodyA = motorpane;
        revoluteJointDef.bodyB = pin;
        revoluteJointDef.collideConnected = false;
        revoluteJointDef.localAnchorA.Set(0,0);
        revoluteJointDef.localAnchorB.Set(0,0);
        revoluteJointDef.enableMotor = true;
        revoluteJointDef.maxMotorTorque = 1000.0f;
        revoluteJointDef.motorSpeed = 4.5f;
        b2RevoluteJoint* m_joint = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef );
    }

    ////////////////////////// W ends  //////////////////////////

    //////////////////////////////G starts /////////////////////////
//G part
    {
        b2PolygonShape shape;
        shape.SetAsBox(0.25f, 4.5f);

        b2BodyDef bd;
        bd.position.Set(29.0f, 30.0f);
        b2Body* g1 = m_world->CreateBody(&bd);
        g1->CreateFixture(&shape, 0.0f);

        shape.SetAsBox(4.0f,0.25f);
        bd.position.Set(33.0f, 25.5f);
        b2Body* g2 = m_world->CreateBody(&bd);
        g2->CreateFixture(&shape, 0.0f);

        const b2Vec2 center(33.0f,41.0f);
        shape.SetAsBox(4.0f,0.25f,center,0.0f);
        b2BodyDef bd2;
        b2Body* stop = m_world->CreateBody(&bd2);
        stop->CreateFixture(&shape, 10.0f);


        shape.SetAsBox(0.6f, 3.7f);
        bd.position.Set(38.0f, 29.0f);
        b2Body* g3 = m_world->CreateBody(&bd);
        g3->CreateFixture(&shape, 0.0f);

        shape.SetAsBox(1.0f, 1.0f);
        bd.position.Set(38.0f, 33.0f);
        bd.type = b2_dynamicBody;
        b2Body* g4 = m_world->CreateBody(&bd);
        g4->CreateFixture(&shape, 0.0f);





    }


    //G-Door
    {
        b2Body* top;
        {
            b2CircleShape cshape;
            b2BodyDef bd;
            cshape.m_radius = 0.4;
            bd.position.Set(29.0f, 38.0f);
            top = m_world->CreateBody(&bd);
            top->CreateFixture(&cshape, 0.5f);
        }
        b2Body* door;
        {
            b2PolygonShape shape;
            shape.SetAsBox(0.25f,2.5f);
            b2BodyDef doordef;
            doordef.type=b2_dynamicBody;
            doordef.position.Set(29.0f,38.0f);
            door=m_world->CreateBody(&doordef);
            door->CreateFixture(&shape,0.25f);
        }

        b2RevoluteJointDef revoluteJointDef;
        revoluteJointDef.bodyA = top;
        revoluteJointDef.bodyB = door;
        revoluteJointDef.collideConnected = false;
        revoluteJointDef.localAnchorA.Set(0.0f,0.0f);//the top right corner of the box
        revoluteJointDef.localAnchorB.Set(0.0f,0.0f);//center of the circle

        b2RevoluteJoint* m_joint2 = (b2RevoluteJoint*)m_world->CreateJoint( &revoluteJointDef );
    }

    //////////////////////////////G ends  //////////////////////////


    ////////////////////////// A starts ////////////////////////////
    //start of code for A

    //middle horizontal shelf

    {
        b2PolygonShape shape;
        shape.SetAsBox(6.0f, 0.15f, b2Vec2(-20.f,20.f), 0.0f);

        b2BodyDef bd;
        bd.position.Set(35.0f, 13.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);
    }
//firstdown1 from the middle hor shelf
    {
        b2PolygonShape shape;
        shape.SetAsBox(1.0f, 0.15f, b2Vec2(-20.f,20.f), 0.0f);

        b2BodyDef bd;
        bd.position.Set(26.0f, 10.4f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);
    }
//firstdown2 from mid hor shelf
    {
        b2PolygonShape shape;
        shape.SetAsBox(1.0f, 0.15f, b2Vec2(-20.f,20.f), 0.0f);

        b2BodyDef bd;
        bd.position.Set(43.0f, 10.5f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);
    }
//new segment above firstdown2..
     {
        b2PolygonShape shape;
        shape.SetAsBox(1.0f, 0.15f, b2Vec2(-20.f,20.f), 0.0f);

        b2BodyDef bd;
        bd.position.Set(42.0f, 11.5f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);
    }


// new domino for seg above
 {
        b2PolygonShape shape;
        shape.SetAsBox(0.1f, 1.7f,b2Vec2(22.2f, 40.0f),0.0f);

        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 20.0f;
        fd.friction = 0.1f;

        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        //bd.position.Set(26.5f, 30.0f);
        b2Body* body = m_world->CreateBody(&bd);
        body->CreateFixture(&fd);

    }
     //new right bottom segment
    {
        b2PolygonShape shape;
        shape.SetAsBox(1.0f, 0.15f, b2Vec2(-20.f,20.f), 0.0f);

        b2BodyDef bd;
        bd.position.Set(47.5f, 7.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);
    }
//second1 from mid hor shelf
   {
        b2PolygonShape shape;
        shape.SetAsBox(1.0f, 0.15f, b2Vec2(-20.f,20.f), 0.0f);

        b2BodyDef bd;
        bd.position.Set(45.0f, 9.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);
    }

//second2 from mid hor shelf
    {
    	b2Body* b18;
    {
        b2PolygonShape shape;
     //   shape.SetAsBox(1.0f, 0.15f, b2Vec2(-20.f,20.f), 0.0f);
        shape.SetAsBox(2.0f, 0.15f);
        b2BodyDef bd;
        bd.position.Set(5.0f, 28.1f);
     //   b2Body* ground = m_world->CreateBody(&bd);
     //   ground->CreateFixture(&shape, 0.0f);
        b18 = m_world->CreateBody(&bd);
        b18->CreateFixture(&shape, 0.0f);
    }
   /* b2Body* b19;
     {
        b2PolygonShape shape;
        shape.SetAsBox(0.1f, 1.7f);

        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 0.1f;
        fd.friction = 0.1f;

        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.position.Set(6.0f, 29.7f);
      //  b2Body* body = m_world->CreateBody(&bd);
        //body->CreateFixture(&fd);
        b19 = m_world->CreateBody(&bd);
        b19->CreateFixture(&shape, 0.0f);

    }*/
    /*b2RevoluteJointDef jd;
        b2Vec2 anchor;
        anchor.Set(6.0f, 27.9f);
        jd.Initialize(b18, b19,anchor);

        m_world->CreateJoint(&jd);*/

    }

//firstup1 from mid hor shelf
    {
        b2PolygonShape shape;
        shape.SetAsBox(0.6f, 0.15f, b2Vec2(-20.f,20.f), 0.0f);

        b2BodyDef bd;
        bd.position.Set(32.3f, 14.4f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);
    }
//firstup2 from mid hor shelf
   {
        b2PolygonShape shape;
        shape.SetAsBox(0.6f, 0.15f, b2Vec2(-20.f,20.f), 0.0f);

        b2BodyDef bd;
        bd.position.Set(38.0f, 15.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);
    }
//top hor shelf
    {
        b2PolygonShape shape;
        shape.SetAsBox(0.6f, 0.15f, b2Vec2(-20.f,20.f), 0.0f);

        b2BodyDef bd;
        bd.position.Set(35.0f, 17.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);
    }

    //Dominos on A
//top domino
    {
        b2PolygonShape shape;
        shape.SetAsBox(0.1f, 1.7f);

        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 50.0f;
        fd.friction = 0.1f;

        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.position.Set(15.0f, 45.0f);
        b2Body* body = m_world->CreateBody(&bd);
        body->CreateFixture(&fd);

    }
//2nd left domino below top
    {
        b2PolygonShape shape;
        shape.SetAsBox(0.1f, 1.7f);

        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 10.0f;
        fd.friction = 0.1f;

        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.position.Set(12.0f, 43.0f);
        b2Body* body = m_world->CreateBody(&bd);
        body->CreateFixture(&fd);

    }
//right to top
  {
        b2PolygonShape shape;
        shape.SetAsBox(0.1f, 1.7f);

        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 10.0f;
        fd.friction = 0.1f;

        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.position.Set(18.0f, 43.0f);
        b2Body* body = m_world->CreateBody(&bd);
        body->CreateFixture(&fd);

    }
//domino on middle slab
    {
        b2PolygonShape shape;
        shape.SetAsBox(0.1f, 1.7f);

        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 10.0f;
        fd.friction = 0.1f;

        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.position.Set(9.5f, 40.0f);
        b2Body* body = m_world->CreateBody(&bd);
        body->CreateFixture(&fd);

    }
//middle right
    {
        b2PolygonShape shape;
        shape.SetAsBox(0.1f, 1.7f);

        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 10.0f;
        fd.friction = 0.1f;

        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.position.Set(20.5f, 40.0f);
        b2Body* body = m_world->CreateBody(&bd);
        body->CreateFixture(&fd);

    }

  {
        b2PolygonShape shape;
        shape.SetAsBox(0.1f, 1.7f);

        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 2.0f;
        fd.friction = 0.1f;

        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.position.Set(5.1f, 36.0f);
        b2Body* body = m_world->CreateBody(&bd);
        body->CreateFixture(&fd);

    }
//rightmost bottommost domino
    {
        b2PolygonShape shape;
        shape.SetAsBox(0.1f, 1.7f,b2Vec2(26.2f, 30.0f),0.15f);

        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 20.0f;
        fd.friction = 0.1f;

        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        //bd.position.Set(26.5f, 30.0f);
        b2Body* body = m_world->CreateBody(&bd);
        body->CreateFixture(&fd);

    }

   {
        b2PolygonShape shape;
        shape.SetAsBox(0.1f, 1.55f);

        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 25.0f;
        fd.friction = 0.1f;

        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.position.Set(4.1f, 32.3f);
        b2Body* body = m_world->CreateBody(&bd);
        body->CreateFixture(&fd);

    }

    {
        b2PolygonShape shape;
        shape.SetAsBox(0.1f, 1.7f);

        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 20.0f;
        fd.friction = 0.1;

        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.position.Set(25.5f, 32.0f);
        b2Body* body = m_world->CreateBody(&bd);
        body->CreateFixture(&fd);

    }
    //left 3rd from bottom
   {
        b2PolygonShape shape;
        shape.SetAsBox(0.1f, 1.7f);

        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 2.0f;
        fd.friction = 0.1f;

        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.position.Set(7.0f, 35.5f);
        b2Body* body = m_world->CreateBody(&bd);
        body->CreateFixture(&fd);
    }
     {
        b2PolygonShape shape;
        shape.SetAsBox(0.3f, 0.15f, b2Vec2(-20.f,20.f), 0.0f);

        b2BodyDef bd;
        bd.position.Set(27.0f, 11.5f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);
    }


   {
        b2PolygonShape shape;
        shape.SetAsBox(0.1f, 1.7f);

        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 30.0f;
        fd.friction = 0.1f;

        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.position.Set(23.5f, 32.5f);
        b2Body* body = m_world->CreateBody(&bd);
        body->CreateFixture(&fd);
    }
     {
        b2PolygonShape shape;
        shape.SetAsBox(1.0f, 0.15f, b2Vec2(-20.f,20.f), 0.0f);

        b2BodyDef bd;
        bd.position.Set(46.0f, 8.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);
        }
    ////////////////////////// A ends  ////////////////////////////

    {
        b2PolygonShape shape;
        const b2Vec2 center(27.0f+10.0f, 14.0f);
        //center.Set(0.0f, 0.0f);
        shape.SetAsBox(2.0f, 0.1f,center,1.57f);

        b2BodyDef bd;
        //bd.position.Set(-31.0f, 30.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);



    }


    {
        b2PolygonShape shape;
        const b2Vec2 center(27.0f+12.0f, 14.0f);
        //center.Set(0.0f, 0.0f);
        shape.SetAsBox(2.0f, 0.1f,center,1.57f);

        b2BodyDef bd;
        //bd.position.Set(-31.0f, 30.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);



    }


    {
        b2PolygonShape shape;
        const b2Vec2 center(27.0f+17.5f, 22.5f);
        //center.Set(0.0f, 0.0f);
        shape.SetAsBox(10.0f, 0.10f,center,0.7f);

        b2BodyDef bd;
        //bd.position.Set(-31.0f, 30.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);



    }
    /* Lower nozzle for wavy path */

    {
        b2PolygonShape shape;
        const b2Vec2 center(27.0f+22.0f, 24.5f);
        //center.Set(0.0f, 0.0f);
        shape.SetAsBox(13.0f, 0.10f,center,0.7f);

        b2BodyDef bd;
        //bd.position.Set(-31.0f, 30.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);



    }

    {
        b2PolygonShape shape;
        const b2Vec2 center(27.0f+22.5f, 31.0f);
        //center.Set(0.0f, 0.0f);
        shape.SetAsBox(1.6f, 0.1f,center,-0.7f);

        b2BodyDef bd;
        //bd.position.Set(-31.0f, 30.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);



    }
    /*Heavy ball*/
    {
        b2Body* spherebody;

        b2CircleShape circle;
        circle.m_radius = 0.4;

        b2FixtureDef ballfd;
        ballfd.shape = &circle;
        ballfd.density = 60.0f;
        ballfd.friction = 0.0f;
        ballfd.restitution = 0.1f;

        b2BodyDef ballbd;
        ballbd.type = b2_dynamicBody;
        ballbd.position.Set(27.0f+18.0f, 33.0f);
        spherebody = m_world->CreateBody(&ballbd);
        spherebody->CreateFixture(&ballfd);

    }
    /*Cap for wavy path*/

    {
        b2PolygonShape shape;
        const b2Vec2 center(26.0f+18.0f, 32.0f);
        //center.Set(0.0f, 0.0f);
        shape.SetAsBox(0.1f, 5.0f,center,-1.57f);

        b2BodyDef bd;
        //bd.position.Set(-31.0f, 30.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);



    }
    /*Lower balls container*/



    {
        b2PolygonShape shape;
        const b2Vec2 center(27.0f+14.0f, 8.0f);
        //center.Set(0.0f, 0.0f);
        shape.SetAsBox(0.1f, 3.0f,center,0.0f);

        b2BodyDef bd;
        //bd.position.Set(-31.0f, 30.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);



    }
    /*Lower plank for box holding balls*/

    {
        b2PolygonShape shape;
        const b2Vec2 center(27.0f+11.0f, 5.0f);
        //center.Set(0.0f, 0.0f);
        shape.SetAsBox(3.0f, 0.1f,center,0.0f);

        b2BodyDef bd;
        //bd.position.Set(-31.0f, 30.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);



    }
    /*Split for ball holding box*/
    {
        b2PolygonShape shape;
        const b2Vec2 center(27.0f+8.0f, 9.5f);
        //center.Set(0.0f, 0.0f);
        shape.SetAsBox(0.1f, 1.2f,center,0.0f);

        b2BodyDef bd;
        //bd.position.Set(-31.0f, 30.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);



    }

    {
        b2PolygonShape shape;
        const b2Vec2 center(27.0f+8.0f, 5.5f);
        //center.Set(0.0f, 0.0f);
        shape.SetAsBox(0.1f, 0.5f,center,0.0f);

        b2BodyDef bd;
        //bd.position.Set(-31.0f, 30.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);



    }
    /*Balls in container*/
    {
        b2Body* spherebody;

        b2CircleShape circle;
        circle.m_radius = 0.25;

        b2FixtureDef ballfd;
        ballfd.shape = &circle;
        ballfd.density = 0.5f;
        ballfd.friction = 0.0f;
        ballfd.restitution = 0.0f;
        for(int j=0; j<22; j++)
        {
            for (int i = 0; i < 9; ++i)
            {
                b2BodyDef ballbd;
                ballbd.type = b2_dynamicBody;
                ballbd.position.Set(27.0f+(11.0+0.1*i), (8.0+0.01*j));
                spherebody = m_world->CreateBody(&ballbd);
                spherebody->CreateFixture(&ballfd);
            }
        }
    }
    /*channel system for balls*/

    {
        b2PolygonShape shape;
        const b2Vec2 center(27.0f+7.0f, 8.0f);
        //center.Set(0.0f, 0.0f);
        shape.SetAsBox(1.0f, 0.1f,center,0.0f);

        b2BodyDef bd;
        //bd.position.Set(-31.0f, 30.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);
    }

    {
        b2PolygonShape shape;
        const b2Vec2 center(27.0f+7.0f, 6.0f);
        //center.Set(0.0f, 0.0f);
        shape.SetAsBox(1.0f, 0.1f,center,0.0f);

        b2BodyDef bd;
        //bd.position.Set(-31.0f, 30.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);
    }

    /*part 2 of channel system*/
    /*split*/
    {
        b2PolygonShape shape;
        const b2Vec2 center(27.0f+6.0f, 9.5f);
        //center.Set(0.0f, 0.0f);
        shape.SetAsBox(0.1f, 1.5f,center,0.0f);

        b2BodyDef bd;
        //bd.position.Set(-31.0f, 30.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);



    }
    {
        b2PolygonShape shape;
        const b2Vec2 center(27.0f+6.0f, 5.5f);
        //center.Set(0.0f, 0.0f);
        shape.SetAsBox(0.1f, 0.5f,center,0.0f);

        b2BodyDef bd;
        //bd.position.Set(-31.0f, 30.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);



    }
    /*funnel*/
    {
        b2Body* gd;
        {
            b2PolygonShape shape;
            const b2Vec2 center(27.0f+4.0f, 11.0f);
            //center.Set(0.0f, 0.0f);
            shape.SetAsBox(2.0f, 0.1f,center,0.1f);

            b2BodyDef bd;
            //bd.position.Set(-31.0f, 30.0f);
            //b2Body* ground = m_world->CreateBody(&bd);
            gd = m_world->CreateBody(&bd);
            gd->CreateFixture(&shape, 0.0f);



        }
    }

    {
        b2PolygonShape shape;
        const b2Vec2 center(27.0f+3.0f, 10.0f);
        //center.Set(0.0f, 0.0f);
        shape.SetAsBox(1.0f, 0.1f,center,0.0f);

        b2BodyDef bd;
        //bd.position.Set(-31.0f, 30.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);



    }
    {
        b2PolygonShape shape;
        const b2Vec2 center(27.0f+4.0f, 7.5f);
        //center.Set(0.0f, 0.0f);
        shape.SetAsBox(0.1f, 2.5f,center,0.0f);

        b2BodyDef bd;
        //bd.position.Set(-31.0f, 30.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);



    }
    /*Lower plank for box holding balls*/

    {
        b2PolygonShape shape;
        const b2Vec2 center(27.0f+5.0f, 5.0f);
        //center.Set(0.0f, 0.0f);
        shape.SetAsBox(1.0f, 0.1f,center,0.0f);

        b2BodyDef bd;
        //bd.position.Set(-31.0f, 30.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);



    }
    /*Heavy weight over balls*/
    {

        b2Body* wtbody;
        b2PolygonShape shape;
        //const b2Vec2 center(27.0f+11.0f, 11.0f);
        //center.Set(0.0f, 0.0f);
        shape.SetAsBox(2.9f, 0.5f);
        b2FixtureDef wtfd;
        wtfd.shape = &shape;
        wtfd.density = 1.0f;
        wtfd.friction = 0.0f;
        wtfd.restitution = 1.0f;
        b2BodyDef bd;
        bd.type=b2_dynamicBody;
        bd.position.Set(27.0f+11.0f, 11.0f);
        wtbody = m_world->CreateBody(&bd);
        wtbody->CreateFixture(&wtfd);
        //b2Body* ground = m_world->CreateBody(&bd);
        //ground->CreateFixture(&shape, 0.0f);
    }

    /*Platform for man*/

    {
        b2PolygonShape shape;
        const b2Vec2 center(27.0f+-3.0f, 5.0f);
        //center.Set(0.0f, 0.0f);
        shape.SetAsBox(3.5f, 1.0f,center,0.0f);

        b2BodyDef bd;
        //bd.position.Set(-31.0f, 30.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);



    }
    /*dish*/
    /*Base of dish*/
    {
        b2PolygonShape shape;
        const b2Vec2 center(27.0f+-0.75f, 8.0f);
        //center.Set(0.0f, 0.0f);
        shape.SetAsBox(1.5f, 0.1f,center,0.0f);

        b2BodyDef bd;
        //bd.position.Set(-31.0f, 30.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);



    }
    // slanting edges of dish
    {
        b2PolygonShape shape;
        const b2Vec2 center(27.0f+-2.2f, 9.1f);
        //center.Set(0.0f, 0.0f);
        shape.SetAsBox(1.2f, 0.1f,center,-1.1f);

        b2BodyDef bd;
        //bd.position.Set(-31.0f, 30.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);



    }
    {
        b2PolygonShape shape;
        const b2Vec2 center(27.0f+1.0f, 9.1f);
        //center.Set(0.0f, 0.0f);
        shape.SetAsBox(1.0f, 0.1f,center,1.1f);

        b2BodyDef bd;
        //bd.position.Set(-31.0f, 30.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);



    }
    /*Block below dish*/
    {
        b2PolygonShape shape;
        const b2Vec2 center(27.0f+-0.75f, 6.9f);
        //center.Set(0.0f, 0.0f);
        shape.SetAsBox(1.5f, 0.9f,center,0.0f);

        b2BodyDef bd;
        //bd.position.Set(-31.0f, 30.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);



    }
    //man
    {
        b2PolygonShape shape;
        const b2Vec2 center(27.0f+-4.9f, 8.5f);
        //center.Set(0.0f, 0.0f);
        shape.SetAsBox(1.5f, 1.5f,center,0.0f);

        b2BodyDef bd;
        //bd.position.Set(-31.0f, 30.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);



    }
    //man's legs

    {
        b2PolygonShape shape;
        const b2Vec2 center(27.0f+-5.1f, 6.5f);
        //center.Set(0.0f, 0.0f);
        shape.SetAsBox(0.1f, 0.5f,center,0.0f);

        b2BodyDef bd;
        //bd.position.Set(-31.0f, 30.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);



    }

    {
        b2PolygonShape shape;
        const b2Vec2 center(27.0f+-4.6f, 6.5f);
        //center.Set(0.0f, 0.0f);
        shape.SetAsBox(0.1f, 0.5f,center,0.0f);

        b2BodyDef bd;
        //bd.position.Set(-31.0f, 30.0f);
        b2Body* ground = m_world->CreateBody(&bd);
        ground->CreateFixture(&shape, 0.0f);



    }

    {
        b2Body* spherebody;

        b2CircleShape circle;
        circle.m_radius = 1.0f;

        b2FixtureDef ballfd;
        ballfd.shape = &circle;
        ballfd.density = 0.5f;
        ballfd.friction = 0.0f;
        ballfd.restitution = 0.0f;
        b2BodyDef ballbd;
        ballbd.position.Set(27.0f+-4.9f, 11.0f);
        spherebody = m_world->CreateBody(&ballbd);
        spherebody->CreateFixture(&ballfd);
    }

}

sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
