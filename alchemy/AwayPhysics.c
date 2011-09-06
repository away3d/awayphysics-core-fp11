
/*
compile: g++ -I./ AwayPhysics.c libbulletdynamics.a libbulletcollision.a libbulletmath.a -O3 -Wall -swc -o AwayPhysics.swc
*/

#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include "AS3.h"
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
//#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "BulletDynamics/Character/btKinematicCharacterController.h"


// trace function sztrace missing in AS3.h
extern "C"
{
    void sztrace(char*);
}

//////////////////////////////////////////////////////////////////////////
/*
*function from Box2DAS http://www.sideroller.com/wck/
*/

/// Macro that allows us to return a pointer to flash without worrying about releasing it.
AS3_Val as3_ptr;
#define return_as3_ptr(ptr) AS3_Release(as3_ptr); as3_ptr = AS3_Ptr(ptr); return as3_ptr;

/// Some macros for easily exporting new / delete functions to AS3.
#define as3_new(type) AS3_Val type##_new(void* data, AS3_Val args) { return_as3_ptr(new type()); };
#define as3_del(type) AS3_Val type##_delete(void* data, AS3_Val args) { type* p; AS3_ArrayValue(args, "PtrType", &p); delete p; return AS3_Null(); };
#define as3_new_del(type) as3_new(type); as3_del(type);

//////////////////////////////////////////////////////////////////////////


btDiscreteDynamicsWorld* dynamicsWorld;

/// create the discrete dynamics world with btDbvtBroadphase
AS3_Val createDiscreteDynamicsWorldWithDbvt(void* data, AS3_Val args) {

	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new	btCollisionDispatcher(collisionConfiguration);
	btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();
	overlappingPairCache->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,overlappingPairCache,solver,collisionConfiguration);

	return_as3_ptr(dynamicsWorld);
}

/// create the discrete dynamics world with btAxisSweep3
AS3_Val createDiscreteDynamicsWorldWithAxisSweep3(void* data, AS3_Val args) {
	double worldAabbMinX;
	double worldAabbMinY;
	double worldAabbMinZ;
	double worldAabbMaxX;
	double worldAabbMaxY;
	double worldAabbMaxZ;
	AS3_ArrayValue(args,"DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType",&worldAabbMinX,&worldAabbMinY,&worldAabbMinZ,&worldAabbMaxX,&worldAabbMaxY,&worldAabbMaxZ);

	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new	btCollisionDispatcher(collisionConfiguration);

	btVector3 worldMin(worldAabbMinX,worldAabbMinY,worldAabbMinZ);
	btVector3 worldMax(worldAabbMaxX,worldAabbMaxY,worldAabbMaxZ);
	btBroadphaseInterface* overlappingPairCache = new btAxisSweep3(worldMin,worldMax);
	overlappingPairCache->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,overlappingPairCache,solver,collisionConfiguration);

	return_as3_ptr(dynamicsWorld);
}

// create a static plane shape
AS3_Val createStaticPlaneShape(void* data, AS3_Val args){
	double normalX;
	double normalY;
	double normalZ;
	double constant;
	AS3_ArrayValue(args,"DoubleType,DoubleType,DoubleType,DoubleType",&normalX,&normalY,&normalZ,&constant);
	
	btCollisionShape* shape = new btStaticPlaneShape(btVector3(normalX,normalY,normalZ),constant);

	return_as3_ptr(shape);
}

// create a cube
AS3_Val createBoxShape(void* data, AS3_Val args){
	double extentsX;
	double extentsY;
	double extentsZ;
	AS3_ArrayValue(args,"DoubleType,DoubleType,DoubleType",&extentsX,&extentsY,&extentsZ);

	btCollisionShape* shape = new btBoxShape(btVector3(extentsX/2,extentsY/2,extentsZ/2));

	return_as3_ptr(shape);
}

// create a sphere
AS3_Val createSphereShape(void* data, AS3_Val args){
	double radius;
	AS3_ArrayValue(args,"DoubleType",&radius);

	btCollisionShape* shape =  new btSphereShape(radius);

	return_as3_ptr(shape);
}

// create a cylinder
AS3_Val createCylinderShape(void* data, AS3_Val args){
	double extentsX;
	double extentsY;
	double extentsZ;
	AS3_ArrayValue(args,"DoubleType,DoubleType,DoubleType",&extentsX,&extentsY,&extentsZ);

	btCollisionShape* shape =  new btCylinderShape(btVector3(extentsX/2,extentsY/2,extentsZ/2));

	return_as3_ptr(shape);
}

// create a capsule
AS3_Val createCapsuleShape(void* data, AS3_Val args){
	double radius;
	double height;
	AS3_ArrayValue(args,"DoubleType,DoubleType",&radius,&height);
	
	btCollisionShape* shape =  new btCapsuleShape(radius,height);
	
	return_as3_ptr(shape);
}

// create a cone
AS3_Val createConeShape(void* data, AS3_Val args){
	double radius;
	double height;
	AS3_ArrayValue(args,"DoubleType,DoubleType",&radius,&height);
	
	btCollisionShape* shape =  new btConeShape(radius,height);
	
	return_as3_ptr(shape);
}

// create a compound shape
AS3_Val createCompoundShape(void* data, AS3_Val args){

	btCollisionShape* shape =  new btCompoundShape();
	
	return_as3_ptr(shape);
}

//add a child shape to compound shape
AS3_Val addCompoundChild(void* data, AS3_Val args){
	btCompoundShape* cshape;
	btCollisionShape* shape;
	double posx,posy,posz;
	double col11,col12,col13,col21,col22,col23,col31,col32,col33;
	AS3_ArrayValue(args,"PtrType,PtrType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType",&cshape,&shape,&posx,&posy,&posz,&col11,&col12,&col13,&col21,&col22,&col23,&col31,&col32,&col33);

	btTransform localTrans;
	localTrans.setIdentity();
	localTrans.setOrigin(btVector3(posx,posy,posz));

	btMatrix3x3 rot;
	rot.setValue(col11,col12,col13,col21,col22,col23,col31,col32,col33);
	localTrans.setBasis(rot);

	cshape->addChildShape(localTrans,shape);

	return AS3_Null();
}

//remove a child shape from compound shape by index
AS3_Val removeCompoundChild(void* data, AS3_Val args){
	btCompoundShape* cshape;
	int index;
	AS3_ArrayValue(args, "PtrType,IntType",&cshape,&index);

	cshape->removeChildShapeByIndex(index);

	return AS3_Null();
}

AS3_Val createHeightmapDataBuffer(void* data, AS3_Val args){
	int size;
	AS3_ArrayValue(args, "IntType",&size);

	btScalar* heightmapData = new btScalar[size];

	return_as3_ptr(heightmapData);
}

AS3_Val removeHeightmapDataBuffer(void* data, AS3_Val args){
	btScalar* heightmapData;

	AS3_ArrayValue(args, "PtrType",&heightmapData);

	delete [] heightmapData;

	return AS3_Null();
}

AS3_Val createTerrainShape(void* data, AS3_Val args){
	btScalar* heightmapData;
	int sw;
	int sh;
	double width;
	double length;
	double heightScale;
	double minHeight;
	double maxHeight;
	int flipQuadEdges;

	AS3_ArrayValue(args, "PtrType,IntType,IntType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,IntType",&heightmapData,&sw,&sh,&width,&length,&heightScale,&minHeight,&maxHeight,&flipQuadEdges);

	btHeightfieldTerrainShape* heightFieldShape = new btHeightfieldTerrainShape(sw,sh,heightmapData, heightScale,minHeight, maxHeight,1, PHY_FLOAT,flipQuadEdges==1);
	heightFieldShape->setUseDiamondSubdivision(true);
	heightFieldShape->setLocalScaling(btVector3(width/sw,1,length/sh));
	return_as3_ptr(heightFieldShape);
}

AS3_Val createTriangleIndexDataBuffer(void* data, AS3_Val args){
	int size;
	AS3_ArrayValue(args, "IntType",&size);

	int* indexData = new int[size];

	return_as3_ptr(indexData);
}

AS3_Val removeTriangleIndexDataBuffer(void* data, AS3_Val args){
	int* indexData;

	AS3_ArrayValue(args, "PtrType",&indexData);

	delete [] indexData;

	return AS3_Null();
}

AS3_Val createTriangleVertexDataBuffer(void* data, AS3_Val args){
	int size;
	AS3_ArrayValue(args, "IntType",&size);

	btScalar* vertexData = new btScalar[size];

	return_as3_ptr(vertexData);
}

AS3_Val removeTriangleVertexDataBuffer(void* data, AS3_Val args){
	btScalar* vertexData;

	AS3_ArrayValue(args, "PtrType",&vertexData);

	delete [] vertexData;

	return AS3_Null();
}

AS3_Val createTriangleIndexVertexArray(void* data, AS3_Val args){
	int numTriangles;
	int* indexBase;
	int numVertices;
	btScalar* vertexBase;

	AS3_ArrayValue(args, "IntType,PtrType,IntType,PtrType",&numTriangles,&indexBase,&numVertices,&vertexBase);

	int indexStride = 3*sizeof(int);
	int vertStride = 3*sizeof(btScalar);

	btTriangleIndexVertexArray* indexVertexArrays=new btTriangleIndexVertexArray(numTriangles,indexBase,indexStride,numVertices,vertexBase,vertStride);

	return_as3_ptr(indexVertexArrays);
}

AS3_Val createBvhTriangleMeshShape(void* data, AS3_Val args){
	btTriangleIndexVertexArray* indexVertexArrays;
	int useQuantizedAabbCompression;
	int buildBvh;

	AS3_ArrayValue(args, "PtrType,IntType,IntType",&indexVertexArrays,&useQuantizedAabbCompression,&buildBvh);

	btBvhTriangleMeshShape* bvhTriangleMesh=new btBvhTriangleMeshShape(indexVertexArrays,useQuantizedAabbCompression==1,buildBvh==1);

	return_as3_ptr(bvhTriangleMesh);
}

AS3_Val createConvexHullShape(void* data, AS3_Val args){
	int numPoints;
	btScalar* points;

	AS3_ArrayValue(args, "IntType,PtrType",&numPoints,&points);

	btConvexHullShape* convexHullShape=new btConvexHullShape(points, numPoints, sizeof(btScalar) * 3);

	return_as3_ptr(convexHullShape);
}

AS3_Val createGImpactMeshShape(void* data, AS3_Val args){
	btTriangleIndexVertexArray* indexVertexArrays;

	AS3_ArrayValue(args, "PtrType",&indexVertexArrays);

	btGImpactMeshShape* gimpactMesh = new btGImpactMeshShape(indexVertexArrays);
	gimpactMesh->updateBound();

	//btCollisionDispatcher * dispatcher = static_cast<btCollisionDispatcher *>(dynamicsWorld ->getDispatcher());
	//btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);

	return_as3_ptr(gimpactMesh);
}

AS3_Val createTriangleShape(void* data, AS3_Val args){
	AS3_Val p0;
	AS3_Val p1;
	AS3_Val p2;

	AS3_ArrayValue(args, "AS3ValType,AS3ValType,AS3ValType",&p0,&p1,&p2);

	double p0x,p0y,p0z;
	p0x=AS3_NumberValue( AS3_GetS( p0, "x" ) );
	p0y=AS3_NumberValue( AS3_GetS( p0, "y" ) );
	p0z=AS3_NumberValue( AS3_GetS( p0, "z" ) );

	double p1x,p1y,p1z;
	p1x=AS3_NumberValue( AS3_GetS( p1, "x" ) );
	p1y=AS3_NumberValue( AS3_GetS( p1, "y" ) );
	p1z=AS3_NumberValue( AS3_GetS( p1, "z" ) );

	double p2x,p2y,p2z;
	p2x=AS3_NumberValue( AS3_GetS( p2, "x" ) );
	p2y=AS3_NumberValue( AS3_GetS( p2, "y" ) );
	p2z=AS3_NumberValue( AS3_GetS( p2, "z" ) );

	btTriangleShapeEx* triangleShape=new btTriangleShapeEx(btVector3(p0x,p0y,p0z),btVector3(p1x,p1y,p1z),btVector3(p2x,p2y,p2z));

	AS3_Release( p0 );
	AS3_Release( p1 );
	AS3_Release( p2 );

	return_as3_ptr(triangleShape);
}

// create rigidbody
AS3_Val createBody(void* data, AS3_Val args){
	AS3_Val as3_Body;
	btCollisionShape* shape;
	double mass;
	
	AS3_ArrayValue(args, "AS3ValType,PtrType,DoubleType",&as3_Body,&shape,&mass);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
	if (isDynamic)
		shape->calculateLocalInertia(mass,localInertia);

	btDefaultMotionState* myMotionState = new btDefaultMotionState();
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);
	body->setUserPointer(as3_Body);

	return_as3_ptr(body);
}

//add the body to the dynamics world
AS3_Val addBody(void* data, AS3_Val args){
	btRigidBody* body;
	AS3_ArrayValue(args, "PtrType", &body);

	dynamicsWorld->addRigidBody(body);

	return AS3_Null();
}

//add a body to the dynamics world with group and mask
AS3_Val addBodyWithGroup(void* data, AS3_Val args){
	btRigidBody* body;
	int group;
	int mask;
	AS3_ArrayValue(args, "PtrType,IntType,IntType", &body, &group, &mask);

	dynamicsWorld->addRigidBody(body,group,mask);

	return AS3_Null();
}

/// remove rigidbody
AS3_Val removeBody(void* data, AS3_Val args){
	btRigidBody* body;
	AS3_ArrayValue(args, "PtrType", &body);

	dynamicsWorld->removeRigidBody(body);

	return AS3_Null();
}

//create a btPoint2PointConstraint with one rigidbody
AS3_Val createP2PConstraint1(void* data, AS3_Val args){
	btRigidBody* bodyA;
	double posAx,posAy,posAz;
	AS3_ArrayValue(args, "PtrType,DoubleType,DoubleType,DoubleType",&bodyA,&posAx,&posAy,&posAz);

	btVector3 pivotInA(posAx,posAy,posAz);

	btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*bodyA,pivotInA);

	return_as3_ptr(p2p);
}

//create a btPoint2PointConstraint between tow rigidbodies
AS3_Val createP2PConstraint2(void* data, AS3_Val args){
	btRigidBody* bodyA;
	btRigidBody* bodyB;
	double posAx,posAy,posAz;
	double posBx,posBy,posBz;
	AS3_ArrayValue(args, "PtrType,PtrType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType",&bodyA,&bodyB,&posAx,&posAy,&posAz,&posBx,&posBy,&posBz);

	btVector3 pivotInA(posAx,posAy,posAz);
	btVector3 pivotInB(posBx,posBy,posBz);

	btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*bodyA,*bodyB,pivotInA,pivotInB);

	return_as3_ptr(p2p);
}

AS3_Val createHingeConstraint1(void* data, AS3_Val args){
	btRigidBody* bodyA;
	double pivotInAx,pivotInAy,pivotInAz;
	double axisInAx,axisInAy,axisInAz;
	int useReferenceFrameA;
	AS3_ArrayValue(args, "PtrType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,IntType",&bodyA,&pivotInAx,&pivotInAy,&pivotInAz,&axisInAx,&axisInAy,&axisInAz,&useReferenceFrameA);
	
	btVector3 pivotInA(pivotInAx,pivotInAy,pivotInAz);
	btVector3 axisInA(axisInAx,axisInAy,axisInAz);

	btHingeConstraint* hinge = new btHingeConstraint(*bodyA, pivotInA, axisInA,useReferenceFrameA==1);

	return_as3_ptr(hinge);
}

AS3_Val createHingeConstraint2(void* data, AS3_Val args){
	btRigidBody* bodyA;
	btRigidBody* bodyB;
	double pivotInAx,pivotInAy,pivotInAz;
	double pivotInBx,pivotInBy,pivotInBz;
	double axisInAx,axisInAy,axisInAz;
	double axisInBx,axisInBy,axisInBz;
	int useReferenceFrameA;
	AS3_ArrayValue(args, "PtrType,PtrType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,IntType",&bodyA,&bodyB,&pivotInAx,&pivotInAy,&pivotInAz,&pivotInBx,&pivotInBy,&pivotInBz,&axisInAx,&axisInAy,&axisInAz,&axisInBx,&axisInBy,&axisInBz,&useReferenceFrameA);
	
	btVector3 pivotInA(pivotInAx,pivotInAy,pivotInAz);
	btVector3 pivotInB(pivotInBx,pivotInBy,pivotInBz);
	btVector3 axisInA(axisInAx,axisInAy,axisInAz);
	btVector3 axisInB(axisInBx,axisInBy,axisInBz);

	btHingeConstraint* hinge = new btHingeConstraint(*bodyA,*bodyB, pivotInA,pivotInB, axisInA,axisInB,useReferenceFrameA==1);

	return_as3_ptr(hinge);
}

AS3_Val createConeTwistConstraint1(void* data, AS3_Val args){
	btRigidBody* bodyA;
	double pivotInAx,pivotInAy,pivotInAz;
	double col11,col12,col13,col21,col22,col23,col31,col32,col33;
	AS3_ArrayValue(args, "PtrType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType",&bodyA,&pivotInAx,&pivotInAy,&pivotInAz,&col11,&col12,&col13,&col21,&col22,&col23,&col31,&col32,&col33);

	btTransform frameInA;
	frameInA.setIdentity();
	frameInA.setOrigin(btVector3(pivotInAx, pivotInAy, pivotInAz));

	btMatrix3x3 rot;
	rot.setValue(col11,col12,col13,col21,col22,col23,col31,col32,col33);
	frameInA.setBasis(rot);

	btConeTwistConstraint* coneTwist=new btConeTwistConstraint(*bodyA,frameInA);

	return_as3_ptr(coneTwist);
}

AS3_Val createConeTwistConstraint2(void* data, AS3_Val args){
	btRigidBody* bodyA;
	AS3_Val pivotInA;
	AS3_Val rotationInA1;
	AS3_Val rotationInA2;
	AS3_Val rotationInA3;
	btRigidBody* bodyB;
	AS3_Val pivotInB;
	AS3_Val rotationInB1;
	AS3_Val rotationInB2;
	AS3_Val rotationInB3;
	AS3_ArrayValue(args, "PtrType,AS3ValType,AS3ValType,AS3ValType,AS3ValType,PtrType,AS3ValType,AS3ValType,AS3ValType,AS3ValType",&bodyA,&pivotInA,&rotationInA1,&rotationInA2,&rotationInA3,&bodyB,&pivotInB,&rotationInB1,&rotationInB2,&rotationInB3);

	double pivotInAx,pivotInAy,pivotInAz;
	pivotInAx=AS3_NumberValue( AS3_GetS( pivotInA, "x" ) );
	pivotInAy=AS3_NumberValue( AS3_GetS( pivotInA, "y" ) );
	pivotInAz=AS3_NumberValue( AS3_GetS( pivotInA, "z" ) );

	double col11,col12,col13,col21,col22,col23,col31,col32,col33;
	col11=AS3_NumberValue( AS3_GetS( rotationInA1, "x" ) );
	col12=AS3_NumberValue( AS3_GetS( rotationInA1, "y" ) );
	col13=AS3_NumberValue( AS3_GetS( rotationInA1, "z" ) );
	col21=AS3_NumberValue( AS3_GetS( rotationInA2, "x" ) );
	col22=AS3_NumberValue( AS3_GetS( rotationInA2, "y" ) );
	col23=AS3_NumberValue( AS3_GetS( rotationInA2, "z" ) );
	col31=AS3_NumberValue( AS3_GetS( rotationInA3, "x" ) );
	col32=AS3_NumberValue( AS3_GetS( rotationInA3, "y" ) );
	col33=AS3_NumberValue( AS3_GetS( rotationInA3, "z" ) );

	btTransform frameInA;
	frameInA.setIdentity();
	frameInA.setOrigin(btVector3(pivotInAx, pivotInAy, pivotInAz));

	btMatrix3x3 rot1;
	rot1.setValue(col11,col12,col13,col21,col22,col23,col31,col32,col33);
	frameInA.setBasis(rot1);


	pivotInAx=AS3_NumberValue( AS3_GetS( pivotInB, "x" ) );
	pivotInAy=AS3_NumberValue( AS3_GetS( pivotInB, "y" ) );
	pivotInAz=AS3_NumberValue( AS3_GetS( pivotInB, "z" ) );

	col11=AS3_NumberValue( AS3_GetS( rotationInB1, "x" ) );
	col12=AS3_NumberValue( AS3_GetS( rotationInB1, "y" ) );
	col13=AS3_NumberValue( AS3_GetS( rotationInB1, "z" ) );
	col21=AS3_NumberValue( AS3_GetS( rotationInB2, "x" ) );
	col22=AS3_NumberValue( AS3_GetS( rotationInB2, "y" ) );
	col23=AS3_NumberValue( AS3_GetS( rotationInB2, "z" ) );
	col31=AS3_NumberValue( AS3_GetS( rotationInB3, "x" ) );
	col32=AS3_NumberValue( AS3_GetS( rotationInB3, "y" ) );
	col33=AS3_NumberValue( AS3_GetS( rotationInB3, "z" ) );

	btTransform frameInB;
	frameInB.setIdentity();
	frameInB.setOrigin(btVector3(pivotInAx, pivotInAy, pivotInAz));

	btMatrix3x3 rot2;
	rot2.setValue(col11,col12,col13,col21,col22,col23,col31,col32,col33);
	frameInB.setBasis(rot2);

	btConeTwistConstraint* coneTwist=new btConeTwistConstraint(*bodyA,*bodyB,frameInA,frameInB);

	AS3_Release( pivotInA );
	AS3_Release( rotationInA1 );
	AS3_Release( rotationInA2 );
	AS3_Release( rotationInA3 );
	AS3_Release( pivotInB );
	AS3_Release( rotationInB1 );
	AS3_Release( rotationInB2 );
	AS3_Release( rotationInB3 );

	return_as3_ptr(coneTwist);
}

AS3_Val createGeneric6DofConstraint1(void* data, AS3_Val args){
	btRigidBody* bodyA;
	double pivotInAx,pivotInAy,pivotInAz;
	double col11,col12,col13,col21,col22,col23,col31,col32,col33;
	int useLinearReferenceFrameA;
	AS3_ArrayValue(args, "PtrType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,IntType",&bodyA,&pivotInAx,&pivotInAy,&pivotInAz,&col11,&col12,&col13,&col21,&col22,&col23,&col31,&col32,&col33,&useLinearReferenceFrameA);

	btTransform frameInA;
	frameInA.setIdentity();
	frameInA.setOrigin(btVector3(pivotInAx, pivotInAy, pivotInAz));

	btMatrix3x3 rot;
	rot.setValue(col11,col12,col13,col21,col22,col23,col31,col32,col33);
	frameInA.setBasis(rot);

	btGeneric6DofConstraint* generic6Dof=new btGeneric6DofConstraint(*bodyA,frameInA,useLinearReferenceFrameA==1);

	return_as3_ptr(generic6Dof);
}

AS3_Val createGeneric6DofConstraint2(void* data, AS3_Val args){
	btRigidBody* bodyA;
	AS3_Val pivotInA;
	AS3_Val rotationInA1;
	AS3_Val rotationInA2;
	AS3_Val rotationInA3;
	btRigidBody* bodyB;
	AS3_Val pivotInB;
	AS3_Val rotationInB1;
	AS3_Val rotationInB2;
	AS3_Val rotationInB3;
	int useLinearReferenceFrameA;
	AS3_ArrayValue(args, "PtrType,AS3ValType,AS3ValType,AS3ValType,AS3ValType,PtrType,AS3ValType,AS3ValType,AS3ValType,AS3ValType,IntType",&bodyA,&pivotInA,&rotationInA1,&rotationInA2,&rotationInA3,&bodyB,&pivotInB,&rotationInB1,&rotationInB2,&rotationInB3,&useLinearReferenceFrameA);

	double pivotInAx,pivotInAy,pivotInAz;
	pivotInAx=AS3_NumberValue( AS3_GetS( pivotInA, "x" ) );
	pivotInAy=AS3_NumberValue( AS3_GetS( pivotInA, "y" ) );
	pivotInAz=AS3_NumberValue( AS3_GetS( pivotInA, "z" ) );

	double col11,col12,col13,col21,col22,col23,col31,col32,col33;
	col11=AS3_NumberValue( AS3_GetS( rotationInA1, "x" ) );
	col12=AS3_NumberValue( AS3_GetS( rotationInA1, "y" ) );
	col13=AS3_NumberValue( AS3_GetS( rotationInA1, "z" ) );
	col21=AS3_NumberValue( AS3_GetS( rotationInA2, "x" ) );
	col22=AS3_NumberValue( AS3_GetS( rotationInA2, "y" ) );
	col23=AS3_NumberValue( AS3_GetS( rotationInA2, "z" ) );
	col31=AS3_NumberValue( AS3_GetS( rotationInA3, "x" ) );
	col32=AS3_NumberValue( AS3_GetS( rotationInA3, "y" ) );
	col33=AS3_NumberValue( AS3_GetS( rotationInA3, "z" ) );

	btTransform frameInA;
	frameInA.setIdentity();
	frameInA.setOrigin(btVector3(pivotInAx, pivotInAy, pivotInAz));

	btMatrix3x3 rot1;
	rot1.setValue(col11,col12,col13,col21,col22,col23,col31,col32,col33);
	frameInA.setBasis(rot1);


	pivotInAx=AS3_NumberValue( AS3_GetS( pivotInB, "x" ) );
	pivotInAy=AS3_NumberValue( AS3_GetS( pivotInB, "y" ) );
	pivotInAz=AS3_NumberValue( AS3_GetS( pivotInB, "z" ) );

	col11=AS3_NumberValue( AS3_GetS( rotationInB1, "x" ) );
	col12=AS3_NumberValue( AS3_GetS( rotationInB1, "y" ) );
	col13=AS3_NumberValue( AS3_GetS( rotationInB1, "z" ) );
	col21=AS3_NumberValue( AS3_GetS( rotationInB2, "x" ) );
	col22=AS3_NumberValue( AS3_GetS( rotationInB2, "y" ) );
	col23=AS3_NumberValue( AS3_GetS( rotationInB2, "z" ) );
	col31=AS3_NumberValue( AS3_GetS( rotationInB3, "x" ) );
	col32=AS3_NumberValue( AS3_GetS( rotationInB3, "y" ) );
	col33=AS3_NumberValue( AS3_GetS( rotationInB3, "z" ) );

	btTransform frameInB;
	frameInB.setIdentity();
	frameInB.setOrigin(btVector3(pivotInAx, pivotInAy, pivotInAz));

	btMatrix3x3 rot2;
	rot2.setValue(col11,col12,col13,col21,col22,col23,col31,col32,col33);
	frameInB.setBasis(rot2);

	btGeneric6DofConstraint* generic6Dof=new btGeneric6DofConstraint(*bodyA,*bodyB,frameInA,frameInB,useLinearReferenceFrameA==1);

	AS3_Release( pivotInA );
	AS3_Release( rotationInA1 );
	AS3_Release( rotationInA2 );
	AS3_Release( rotationInA3 );
	AS3_Release( pivotInB );
	AS3_Release( rotationInB1 );
	AS3_Release( rotationInB2 );
	AS3_Release( rotationInB3 );

	return_as3_ptr(generic6Dof);
}

//add a constraint to the dynamics world
AS3_Val addConstraint(void* data, AS3_Val args){
	btTypedConstraint* constraint;
	int disableCollisions;
	AS3_ArrayValue(args, "PtrType,IntType", &constraint, &disableCollisions);

	dynamicsWorld->addConstraint(constraint,disableCollisions==1);
	
	return AS3_Null();
}

/// remove constraint
AS3_Val removeConstraint(void* data, AS3_Val args){
	btTypedConstraint* constraint;
	AS3_ArrayValue(args, "PtrType", &constraint);

	dynamicsWorld->removeConstraint(constraint);

	return AS3_Null();
}

AS3_Val createVehicle(void* data, AS3_Val args) {
	AS3_Val tuning;
	btRigidBody* chassis;
	AS3_ArrayValue(args, "AS3ValType,PtrType",&tuning,&chassis);

	btRaycastVehicle::btVehicleTuning m_tuning;
	m_tuning.m_suspensionStiffness=AS3_NumberValue( AS3_GetS( tuning, "suspensionStiffness" ) );
	m_tuning.m_suspensionCompression=AS3_NumberValue( AS3_GetS( tuning, "suspensionCompression" ) );
	m_tuning.m_suspensionDamping=AS3_NumberValue( AS3_GetS( tuning, "suspensionDamping" ) );
	m_tuning.m_maxSuspensionTravelCm=AS3_NumberValue( AS3_GetS( tuning, "maxSuspensionTravelCm" ) );
	m_tuning.m_frictionSlip=AS3_NumberValue( AS3_GetS( tuning, "frictionSlip" ) );
	m_tuning.m_maxSuspensionForce=AS3_NumberValue( AS3_GetS( tuning, "maxSuspensionForce" ) );

	btVehicleRaycaster*	m_vehicleRayCaster = new btDefaultVehicleRaycaster(dynamicsWorld);
	btRaycastVehicle* m_vehicle = new btRaycastVehicle(m_tuning,chassis,m_vehicleRayCaster);
	m_vehicle->setCoordinateSystem(0,1,2);

	AS3_Release( tuning );

	return_as3_ptr(m_vehicle);
}

AS3_Val addVehicleWheel(void* data, AS3_Val args){
	btRaycastVehicle* m_vehicle;
	double connectionPointCS0X,connectionPointCS0Y,connectionPointCS0Z;
	double wheelDirectionCS0X,wheelDirectionCS0Y,wheelDirectionCS0Z;
	double wheelAxleCSX,wheelAxleCSY,wheelAxleCSZ;
	double suspensionRestLength;
	double wheelRadius;
	AS3_Val tuning;
	int isFrontWheel;
	AS3_ArrayValue(args, "PtrType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,DoubleType,AS3ValType,IntType",&m_vehicle,&connectionPointCS0X,&connectionPointCS0Y,&connectionPointCS0Z,&wheelDirectionCS0X,&wheelDirectionCS0Y,&wheelDirectionCS0Z,&wheelAxleCSX,&wheelAxleCSY,&wheelAxleCSZ,&suspensionRestLength,&wheelRadius,&tuning,&isFrontWheel);

	btVector3 connectionPointCS0(connectionPointCS0X,connectionPointCS0Y,connectionPointCS0Z);
	btVector3 wheelDirectionCS0(wheelDirectionCS0X,wheelDirectionCS0Y,wheelDirectionCS0Z);
	btVector3 wheelAxleCS(wheelAxleCSX,wheelAxleCSY,wheelAxleCSZ);

	btRaycastVehicle::btVehicleTuning m_tuning;
	m_tuning.m_suspensionStiffness=AS3_NumberValue( AS3_GetS( tuning, "suspensionStiffness" ) );
	m_tuning.m_suspensionCompression=AS3_NumberValue( AS3_GetS( tuning, "suspensionCompression" ) );
	m_tuning.m_suspensionDamping=AS3_NumberValue( AS3_GetS( tuning, "suspensionDamping" ) );
	m_tuning.m_maxSuspensionTravelCm=AS3_NumberValue( AS3_GetS( tuning, "maxSuspensionTravelCm" ) );
	m_tuning.m_frictionSlip=AS3_NumberValue( AS3_GetS( tuning, "frictionSlip" ) );
	m_tuning.m_maxSuspensionForce=AS3_NumberValue( AS3_GetS( tuning, "maxSuspensionForce" ) );

	m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,suspensionRestLength,wheelRadius,m_tuning,isFrontWheel==1);

	AS3_Release( tuning );

	return_as3_ptr(&m_vehicle->getWheelInfo(m_vehicle->getNumWheels()-1));
}

AS3_Val addVehicle(void* data, AS3_Val args){
	btActionInterface* vehicle;
	AS3_ArrayValue(args, "PtrType", &vehicle);

	dynamicsWorld->addVehicle(vehicle);

	return AS3_Null();
}

AS3_Val removeVehicle(void* data, AS3_Val args){
	btActionInterface* vehicle;
	AS3_ArrayValue(args, "PtrType", &vehicle);

	dynamicsWorld->removeVehicle(vehicle);

	return AS3_Null();
}

AS3_Val createGhostObject(void* data, AS3_Val args){
	AS3_Val as3_Body;
	btCollisionShape* shape;

	AS3_ArrayValue(args, "AS3ValType,PtrType",&as3_Body,&shape);

	btPairCachingGhostObject* ghostObject = new btPairCachingGhostObject();
	ghostObject->setCollisionShape(shape);
	ghostObject->setUserPointer(as3_Body);

	return_as3_ptr(ghostObject);
}

AS3_Val createCharacter(void* data, AS3_Val args){
	btPairCachingGhostObject* ghostObject;
	btConvexShape* shape;
	double stepHeight;
	int upAxis;

	AS3_ArrayValue(args, "PtrType,PtrType,DoubleType,IntType",&ghostObject,&shape,&stepHeight,&upAxis);

	btKinematicCharacterController* character = new btKinematicCharacterController (ghostObject,shape,stepHeight,upAxis);

	return_as3_ptr(character);
}

AS3_Val addCharacter(void* data, AS3_Val args){
	btKinematicCharacterController* character;
	int group;
	int mask;
	AS3_ArrayValue(args, "PtrType,IntType,IntType", &character,&group,&mask);

	dynamicsWorld->addCollisionObject(character->m_ghostObject,group,mask);
	dynamicsWorld->addCharacter(character);

	return AS3_Null();
}

AS3_Val removeCharacter(void* data, AS3_Val args){
	btKinematicCharacterController* character;
	AS3_ArrayValue(args, "PtrType", &character);

	dynamicsWorld->removeCollisionObject(character->m_ghostObject);
	dynamicsWorld->removeCharacter(character);

	return AS3_Null();
}

/// physic step
AS3_Val step(void* data, AS3_Val args) {
	double timestep;
	int maxsubstep;
	double fixedtime;
	AS3_ArrayValue(args, "DoubleType,IntType,DoubleType",&timestep,&maxsubstep,&fixedtime);
	dynamicsWorld->stepSimulation(timestep,maxsubstep,fixedtime);

	int vehiclesLen=dynamicsWorld->m_vehicles.size();
	for (int i=0;i<vehiclesLen;i++)
	{
		btRaycastVehicle* vehicle=(btRaycastVehicle*)dynamicsWorld->m_vehicles[i];
		int wheelLen=vehicle->getNumWheels();
		for (int j=0;j<wheelLen;j++){
			vehicle->updateWheelTransform(j,true);
		}
	}

	if(dynamicsWorld->m_collisionCallbackOn){
		int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
		for (int i=0;i<numManifolds;i++)
		{
			btPersistentManifold* contactManifold = dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
			btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
			btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());

			if (obA->getCollisionFlags() & btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK){
				int numContacts = contactManifold->getNumContacts();
				if(numContacts>0){
					btManifoldPoint* mpt=new btManifoldPoint();
					mpt->m_localPointA=btVector3(0,0,0);
					mpt->m_localPointB=btVector3(0,0,0);
					mpt->m_normalWorldOnB=btVector3(0,0,0);
					mpt->m_appliedImpulse=0;
					for (int j=0;j<numContacts;j++)
					{
						btManifoldPoint& pt = contactManifold->getContactPoint(j);
						mpt->m_localPointA+=pt.m_localPointA;
						mpt->m_localPointB+=pt.m_localPointB;
						mpt->m_normalWorldOnB+=pt.m_normalWorldOnB;
						mpt->m_appliedImpulse+=pt.m_appliedImpulse;
					}
					mpt->m_localPointA/=numContacts;
					mpt->m_localPointB/=numContacts;
					mpt->m_normalWorldOnB.normalize();
					mpt->m_appliedImpulse/=numContacts;

					AS3_CallTS("collisionCallback",(AS3_Val)obA->getUserPointer(), "PtrType, AS3ValType", mpt,(AS3_Val)obB->getUserPointer());

					delete mpt;
				}
			}

			if(obB->getCollisionFlags() & btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK){
				int numContacts = contactManifold->getNumContacts();
				if(numContacts>0){
					btManifoldPoint* mpt=new btManifoldPoint();
					mpt->m_localPointA=btVector3(0,0,0);
					mpt->m_localPointB=btVector3(0,0,0);
					mpt->m_normalWorldOnB=btVector3(0,0,0);
					mpt->m_appliedImpulse=0;
					for (int j=0;j<numContacts;j++)
					{
						btManifoldPoint& pt = contactManifold->getContactPoint(j);
						mpt->m_localPointA+=pt.m_localPointB;
						mpt->m_localPointB+=pt.m_localPointA;
						mpt->m_normalWorldOnB+=pt.m_normalWorldOnB;
						mpt->m_appliedImpulse+=pt.m_appliedImpulse;
					}
					mpt->m_localPointA/=numContacts;
					mpt->m_localPointB/=numContacts;
					mpt->m_normalWorldOnB/=-1;
					mpt->m_normalWorldOnB.normalize();
					mpt->m_appliedImpulse/=numContacts;

					AS3_CallTS("collisionCallback",(AS3_Val)obB->getUserPointer(), "PtrType, AS3ValType", mpt,(AS3_Val)obA->getUserPointer());

					delete mpt;
				}
			}
		}
	}

	return AS3_Null();
}

int main() {

	AS3_Val createDiscreteDynamicsWorldWithDbvtMethod = AS3_Function( NULL, createDiscreteDynamicsWorldWithDbvt );
	AS3_Val createDiscreteDynamicsWorldWithAxisSweep3Method = AS3_Function( NULL, createDiscreteDynamicsWorldWithAxisSweep3 );
	AS3_Val createStaticPlaneShapeMethod = AS3_Function( NULL, createStaticPlaneShape );
	AS3_Val createBoxShapeMethod = AS3_Function( NULL, createBoxShape );
	AS3_Val createSphereShapeMethod = AS3_Function( NULL, createSphereShape );
	AS3_Val createCylinderShapeMethod = AS3_Function( NULL, createCylinderShape );
	AS3_Val createCapsuleShapeMethod = AS3_Function( NULL, createCapsuleShape );
	AS3_Val createConeShapeMethod = AS3_Function( NULL, createConeShape );
	AS3_Val createCompoundShapeMethod = AS3_Function( NULL, createCompoundShape );
	AS3_Val addCompoundChildMethod = AS3_Function( NULL, addCompoundChild );
	AS3_Val removeCompoundChildMethod = AS3_Function( NULL, removeCompoundChild );
	AS3_Val createHeightmapDataBufferMethod = AS3_Function( NULL, createHeightmapDataBuffer );
	AS3_Val removeHeightmapDataBufferMethod = AS3_Function( NULL, removeHeightmapDataBuffer );
	AS3_Val createTerrainShapeMethod = AS3_Function( NULL, createTerrainShape );
	AS3_Val createTriangleIndexDataBufferMethod = AS3_Function( NULL, createTriangleIndexDataBuffer );
	AS3_Val removeTriangleIndexDataBufferMethod = AS3_Function( NULL, removeTriangleIndexDataBuffer );
	AS3_Val createTriangleVertexDataBufferMethod = AS3_Function( NULL, createTriangleVertexDataBuffer );
	AS3_Val removeTriangleVertexDataBufferMethod = AS3_Function( NULL, removeTriangleVertexDataBuffer );
	AS3_Val createTriangleIndexVertexArrayMethod = AS3_Function( NULL, createTriangleIndexVertexArray );
	AS3_Val createBvhTriangleMeshShapeMethod = AS3_Function( NULL, createBvhTriangleMeshShape );
	AS3_Val createConvexHullShapeMethod = AS3_Function( NULL, createConvexHullShape );
	AS3_Val createGImpactMeshShapeMethod = AS3_Function( NULL, createGImpactMeshShape );
	AS3_Val createTriangleShapeMethod = AS3_Function( NULL, createTriangleShape );
	AS3_Val createBodyMethod = AS3_Function( NULL, createBody );
	AS3_Val addBodyWithGroupMethod = AS3_Function( NULL, addBodyWithGroup );
	AS3_Val addBodyMethod = AS3_Function( NULL, addBody );
	AS3_Val removeBodyMethod = AS3_Function( NULL, removeBody );
	AS3_Val createP2PConstraintMethod1 = AS3_Function( NULL, createP2PConstraint1 );
	AS3_Val createP2PConstraintMethod2 = AS3_Function( NULL, createP2PConstraint2 );
	AS3_Val createHingeConstraintMethod1 = AS3_Function( NULL, createHingeConstraint1 );
	AS3_Val createHingeConstraintMethod2 = AS3_Function( NULL, createHingeConstraint2 );
	AS3_Val createConeTwistConstraintMethod1 = AS3_Function( NULL, createConeTwistConstraint1 );
	AS3_Val createConeTwistConstraintMethod2 = AS3_Function( NULL, createConeTwistConstraint2 );
	AS3_Val createGeneric6DofConstraintMethod1 = AS3_Function( NULL, createGeneric6DofConstraint1 );
	AS3_Val createGeneric6DofConstraintMethod2 = AS3_Function( NULL, createGeneric6DofConstraint2 );
	AS3_Val addConstraintMethod = AS3_Function( NULL, addConstraint );
	AS3_Val removeConstraintMethod = AS3_Function( NULL, removeConstraint );
	AS3_Val createVehicleMethod = AS3_Function( NULL, createVehicle );
	AS3_Val addVehicleWheelMethod = AS3_Function( NULL, addVehicleWheel );
	AS3_Val addVehicleMethod = AS3_Function( NULL, addVehicle );
	AS3_Val removeVehicleMethod = AS3_Function( NULL, removeVehicle );
	AS3_Val createGhostObjectMethod = AS3_Function( NULL, createGhostObject );
	AS3_Val createCharacterMethod = AS3_Function( NULL, createCharacter );
	AS3_Val addCharacterMethod = AS3_Function( NULL, addCharacter );
	AS3_Val removeCharacterMethod = AS3_Function( NULL, removeCharacter );
	AS3_Val stepMethod = AS3_Function( NULL, step );

	AS3_Val result = AS3_Object( "createDiscreteDynamicsWorldWithDbvtMethod:AS3ValType,"
								 "createDiscreteDynamicsWorldWithAxisSweep3Method:AS3ValType,"
								 "createStaticPlaneShapeMethod:AS3ValType,"
								 "createBoxShapeMethod:AS3ValType,"
								 "createSphereShapeMethod:AS3ValType,"
								 "createCylinderShapeMethod:AS3ValType,"
								 "createCapsuleShapeMethod:AS3ValType,"
								 "createConeShapeMethod:AS3ValType,"
								 "createCompoundShapeMethod:AS3ValType,"
								 "addCompoundChildMethod:AS3ValType,"
								 "removeCompoundChildMethod:AS3ValType,"
								 "createHeightmapDataBufferMethod:AS3ValType,"
								 "removeHeightmapDataBufferMethod:AS3ValType,"
								 "createTerrainShapeMethod:AS3ValType,"
								 "createTriangleIndexDataBufferMethod:AS3ValType,"
								 "removeTriangleIndexDataBufferMethod:AS3ValType,"
								 "createTriangleVertexDataBufferMethod:AS3ValType,"
								 "removeTriangleVertexDataBufferMethod:AS3ValType,"
								 "createTriangleIndexVertexArrayMethod:AS3ValType,"
								 "createBvhTriangleMeshShapeMethod:AS3ValType,"
								 "createConvexHullShapeMethod:AS3ValType,"
								 "createGImpactMeshShapeMethod:AS3ValType,"
								 "createTriangleShapeMethod:AS3ValType,"
								 "createBodyMethod:AS3ValType,"
								 "addBodyWithGroupMethod:AS3ValType,"
								 "addBodyMethod:AS3ValType,"
								 "removeBodyMethod:AS3ValType,"
								 "createP2PConstraintMethod1:AS3ValType,"
								 "createP2PConstraintMethod2:AS3ValType,"
								 "createHingeConstraintMethod1:AS3ValType,"
								 "createHingeConstraintMethod2:AS3ValType,"
								 "createConeTwistConstraintMethod1:AS3ValType,"
								 "createConeTwistConstraintMethod2:AS3ValType,"
								 "createGeneric6DofConstraintMethod1:AS3ValType,"
								 "createGeneric6DofConstraintMethod2:AS3ValType,"
								 "addConstraintMethod:AS3ValType,"
								 "removeConstraintMethod:AS3ValType,"
								 "createVehicleMethod:AS3ValType,"
								 "addVehicleWheelMethod:AS3ValType,"
								 "addVehicleMethod:AS3ValType,"
								 "removeVehicleMethod:AS3ValType,"
								 "createGhostObjectMethod:AS3ValType,"
								 "createCharacterMethod:AS3ValType,"
								 "addCharacterMethod:AS3ValType,"
								 "removeCharacterMethod:AS3ValType,"
								 "stepMethod:AS3ValType",

								 createDiscreteDynamicsWorldWithDbvtMethod,
								 createDiscreteDynamicsWorldWithAxisSweep3Method,
								 createStaticPlaneShapeMethod,
								 createBoxShapeMethod,
								 createSphereShapeMethod,
								 createCylinderShapeMethod,
								 createCapsuleShapeMethod,
								 createConeShapeMethod,
								 createCompoundShapeMethod,
								 addCompoundChildMethod,
								 removeCompoundChildMethod,
								 createHeightmapDataBufferMethod,
								 removeHeightmapDataBufferMethod,
								 createTerrainShapeMethod,
								 createTriangleIndexDataBufferMethod,
								 removeTriangleIndexDataBufferMethod,
								 createTriangleVertexDataBufferMethod,
								 removeTriangleVertexDataBufferMethod,
								 createTriangleIndexVertexArrayMethod,
								 createBvhTriangleMeshShapeMethod,
								 createConvexHullShapeMethod,
								 createGImpactMeshShapeMethod,
								 createTriangleShapeMethod,
								 createBodyMethod,
								 addBodyWithGroupMethod,
								 addBodyMethod,
								 removeBodyMethod,
								 createP2PConstraintMethod1,
								 createP2PConstraintMethod2,
								 createHingeConstraintMethod1,
								 createHingeConstraintMethod2,
								 createConeTwistConstraintMethod1,
								 createConeTwistConstraintMethod2,
								 createGeneric6DofConstraintMethod1,
								 createGeneric6DofConstraintMethod2,
								 addConstraintMethod,
								 removeConstraintMethod,
								 createVehicleMethod,
								 addVehicleWheelMethod,
								 addVehicleMethod,
								 removeVehicleMethod,
								 createGhostObjectMethod,
								 createCharacterMethod,
								 addCharacterMethod,
								 removeCharacterMethod,
								 stepMethod);

	AS3_Release( createDiscreteDynamicsWorldWithDbvtMethod );
	AS3_Release( createDiscreteDynamicsWorldWithAxisSweep3Method );
	AS3_Release( createStaticPlaneShapeMethod );
	AS3_Release( createBoxShapeMethod );
	AS3_Release( createSphereShapeMethod );
	AS3_Release( createCylinderShapeMethod );
	AS3_Release( createCapsuleShapeMethod );
	AS3_Release( createConeShapeMethod );
	AS3_Release( createCompoundShapeMethod );
	AS3_Release( addCompoundChildMethod );
	AS3_Release( removeCompoundChildMethod );
	AS3_Release( createHeightmapDataBufferMethod );
	AS3_Release( removeHeightmapDataBufferMethod );
	AS3_Release( createTerrainShapeMethod );
	AS3_Release( createTriangleIndexDataBufferMethod );
	AS3_Release( removeTriangleIndexDataBufferMethod );
	AS3_Release( createTriangleVertexDataBufferMethod );
	AS3_Release( removeTriangleVertexDataBufferMethod );
	AS3_Release( createTriangleIndexVertexArrayMethod );
	AS3_Release( createBvhTriangleMeshShapeMethod );
	AS3_Release( createConvexHullShapeMethod );
	AS3_Release( createGImpactMeshShapeMethod );
	AS3_Release( createTriangleShapeMethod );
	AS3_Release( createBodyMethod );
	AS3_Release( addBodyWithGroupMethod );
	AS3_Release( addBodyMethod );
	AS3_Release( removeBodyMethod );
	AS3_Release( createP2PConstraintMethod1 );
	AS3_Release( createP2PConstraintMethod2 );
	AS3_Release( createHingeConstraintMethod1 );
	AS3_Release( createHingeConstraintMethod2 );
	AS3_Release( createConeTwistConstraintMethod1 );
	AS3_Release( createConeTwistConstraintMethod2 );
	AS3_Release( createGeneric6DofConstraintMethod1 );
	AS3_Release( createGeneric6DofConstraintMethod2 );
	AS3_Release( addConstraintMethod );
	AS3_Release( removeConstraintMethod );
	AS3_Release( createVehicleMethod );
	AS3_Release( addVehicleWheelMethod );
	AS3_Release( addVehicleMethod );
	AS3_Release( removeVehicleMethod );
	AS3_Release( createGhostObjectMethod );
	AS3_Release( createCharacterMethod );
	AS3_Release( addCharacterMethod );
	AS3_Release( removeCharacterMethod );
	AS3_Release( stepMethod );

	AS3_LibInit(result);

	return 0; 
}