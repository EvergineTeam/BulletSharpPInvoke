#include "main.h"

#ifndef BT_COLLISION_WORLD_H
#define p_btCollisionWorld_ContactResultCallback_addSingleResult void*
#define p_btCollisionWorld_ContactResultCallback_needsCollision void*
#define p_btCollisionWorld_ConvexResultCallback_addSingleResult void*
#define p_btCollisionWorld_ConvexResultCallback_needsCollision void*
#define p_btCollisionWorld_RayResultCallback_addSingleResult void*
#define p_btCollisionWorld_RayResultCallback_needsCollision void*
#define btCollisionWorld_ContactResultCallbackWrapper void
#define btCollisionWorld_ConvexResultCallbackWrapper void
#define btCollisionWorld_RayResultCallbackWrapper void
#else
typedef btScalar (*p_btCollisionWorld_ContactResultCallback_addSingleResult)(btManifoldPoint& cp,
	const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0, const btCollisionObjectWrapper* colObj1Wrap,
	int partId1, int index1);
typedef bool (*p_btCollisionWorld_ContactResultCallback_needsCollision)(btBroadphaseProxy* proxy0);

class btCollisionWorld_ContactResultCallbackWrapper : public btCollisionWorld_ContactResultCallback
{
private:
	p_btCollisionWorld_ContactResultCallback_addSingleResult _addSingleResultCallback;
	p_btCollisionWorld_ContactResultCallback_needsCollision _needsCollisionCallback;

public:
	btCollisionWorld_ContactResultCallbackWrapper(p_btCollisionWorld_ContactResultCallback_addSingleResult addSingleResultCallback,
		p_btCollisionWorld_ContactResultCallback_needsCollision needsCollisionCallback);

	virtual btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap,
		int partId0, int index0, const btCollisionObjectWrapper* colObj1Wrap, int partId1,
		int index1);
	virtual bool needsCollision(btBroadphaseProxy* proxy0) const;

	virtual bool baseNeedsCollision(btBroadphaseProxy* proxy0) const;
};

typedef btScalar (*p_btCollisionWorld_ConvexResultCallback_addSingleResult)(btCollisionWorld_LocalConvexResult& convexResult,
	bool normalInWorldSpace);
typedef bool (*p_btCollisionWorld_ConvexResultCallback_needsCollision)(btBroadphaseProxy* proxy0);

class btCollisionWorld_ConvexResultCallbackWrapper : public btCollisionWorld_ConvexResultCallback
{
private:
	p_btCollisionWorld_ConvexResultCallback_addSingleResult _addSingleResultCallback;
	p_btCollisionWorld_ConvexResultCallback_needsCollision _needsCollisionCallback;

public:
	btCollisionWorld_ConvexResultCallbackWrapper(p_btCollisionWorld_ConvexResultCallback_addSingleResult addSingleResultCallback,
		p_btCollisionWorld_ConvexResultCallback_needsCollision needsCollisionCallback);

	virtual btScalar addSingleResult(btCollisionWorld_LocalConvexResult& convexResult,
		bool normalInWorldSpace);
	virtual bool needsCollision(btBroadphaseProxy* proxy0) const;

	virtual bool baseNeedsCollision(btBroadphaseProxy* proxy0) const;
};

typedef btScalar (*p_btCollisionWorld_RayResultCallback_addSingleResult)(btCollisionWorld_LocalRayResult& rayResult,
	bool normalInWorldSpace);
typedef bool (*p_btCollisionWorld_RayResultCallback_needsCollision)(btBroadphaseProxy* proxy0);

class btCollisionWorld_RayResultCallbackWrapper : public btCollisionWorld_RayResultCallback
{
private:
	p_btCollisionWorld_RayResultCallback_addSingleResult _addSingleResultCallback;
	p_btCollisionWorld_RayResultCallback_needsCollision _needsCollisionCallback;

public:
	btCollisionWorld_RayResultCallbackWrapper(p_btCollisionWorld_RayResultCallback_addSingleResult addSingleResultCallback,
		p_btCollisionWorld_RayResultCallback_needsCollision needsCollisionCallback);

	virtual btScalar addSingleResult(btCollisionWorld_LocalRayResult& rayResult,
		bool normalInWorldSpace);
	virtual bool needsCollision(btBroadphaseProxy* proxy0) const;

	virtual bool baseNeedsCollision(btBroadphaseProxy* proxy0) const;
};


//struct command_DrawAabb {
//	btVector3* from;
//	btVector3* to;
//	btVector3* color;
//};
//
//struct command_DrawArc {
//	btVector3* center;
//	btVector3* normal;
//	btVector3* axis;
//	float radiusA;
//	float radiusB;
//	float minAngle;
//	float maxAngle;
//	btVector3* color;
//	bool drawSect;
//	float stepDegrees;
//};

class btEvergineDebugDrawWrapper : public btIDebugDraw
{
private:
	void* buffer;
	int bufferCapacity;
	void* currentPointer;
	int currentUsage;
	int drawCommandCount;
	DebugDrawModes debugDrawModes;
	DefaultColors drawColors;

	void ensureCapacity(void** ptr, int size);

public:
	void* _debugDrawGCHandle;
	void* getGCHandle();

	void reset();
	void getDrawInformation(int* commandCount, void** buffer);

	btEvergineDebugDrawWrapper(void* debugDrawGCHandle);
	~btEvergineDebugDrawWrapper();

	virtual void draw3dText(const btVector3& location, const char* textString);
	virtual void drawAabb(const btVector3& from, const btVector3& to, const btVector3& color);
	virtual void drawArc(const btVector3& center, const btVector3& normal, const btVector3& axis,
		btScalar radiusA, btScalar radiusB, btScalar minAngle, btScalar maxAngle, const btVector3& color,
		bool drawSect, btScalar stepDegrees);
	virtual void drawArc(const btVector3& center, const btVector3& normal, const btVector3& axis,
		btScalar radiusA, btScalar radiusB, btScalar minAngle, btScalar maxAngle,
		const btVector3& color, bool drawSect);
	virtual void drawBox(const btVector3& bbMin, const btVector3& bbMax, const btVector3& color);
	virtual void drawBox(const btVector3& bbMin, const btVector3& bbMax, const btTransform& trans,
		const btVector3& color);
	virtual void drawCapsule(btScalar radius, btScalar halfHeight, int upAxis, const btTransform& transform,
		const btVector3& color);
	virtual void drawCone(btScalar radius, btScalar height, int upAxis, const btTransform& transform,
		const btVector3& color);
	virtual void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB,
		btScalar distance, int lifeTime, const btVector3& color);
	virtual void drawCylinder(btScalar radius, btScalar halfHeight, int upAxis, const btTransform& transform,
		const btVector3& color);
	virtual void drawLine(const btVector3& from, const btVector3& to, const btVector3& color);
	virtual void drawPlane(const btVector3& planeNormal, btScalar planeConst, const btTransform& transform,
		const btVector3& color);
	virtual void drawSphere(const btVector3& p, btScalar radius, const btVector3& color);
	virtual void drawSphere(btScalar radius, const btTransform& transform, const btVector3& color);
	virtual void drawSpherePatch(const btVector3& center, const btVector3& up, const btVector3& axis,
		btScalar radius, btScalar minTh, btScalar maxTh, btScalar minPs, btScalar maxPs,
		const btVector3& color, btScalar stepDegrees);
	virtual void drawSpherePatch(const btVector3& center, const btVector3& up, const btVector3& axis, btScalar radius,
		btScalar minTh, btScalar maxTh, btScalar minPs, btScalar maxPs, const btVector3& color);
	virtual void drawTransform(const btTransform& transform, btScalar orthoLen);
	virtual void drawTriangle(const btVector3& v0, const btVector3& v1, const btVector3& v2,
		const btVector3& color, btScalar __unnamed4);
	virtual void drawTriangle(const btVector3& v0, const btVector3& v1, const btVector3& v2,
		const btVector3&, const btVector3&, const btVector3&, const btVector3& color, btScalar alpha);

	virtual void baseDrawAabb(const btVector3& from, const btVector3& to, const btVector3& color);
	virtual void baseDrawCone(btScalar radius, btScalar height, int upAxis, const btTransform& transform, const btVector3& color);
	virtual void baseDrawCylinder(btScalar radius, btScalar halfHeight, int upAxis, const btTransform& transform, const btVector3& color);
	virtual void baseDrawSphere(const btVector3& p, btScalar radius, const btVector3& color);
	virtual void baseDrawTriangle(const btVector3& v0, const btVector3& v1, const btVector3& v2, const btVector3& color, btScalar);
	virtual void baseDrawTriangle(const btVector3& v0, const btVector3& v1, const btVector3& v2,
		const btVector3&, const btVector3&, const btVector3&, const btVector3& color, btScalar alpha);

	virtual void reportErrorWarning(const char* warningString);

	virtual void setDebugMode(int debugMode);
	virtual int	getDebugMode() const;

	virtual DefaultColors	getDefaultColors() const { return drawColors; }
	virtual void setDefaultColors(const DefaultColors& colors) { drawColors = colors; }
};

#endif

#ifdef __cplusplus
extern "C" {
#endif
	EXPORT void* btCollisionWorld_CreateBuffer();
	EXPORT void  btCollisionWorld_DeleteBuffer(void* buffer);
	EXPORT int   btCollisionWorld_ContactTest(btCollisionWorld* world, btCollisionObject* collisionObject, int shapeIndex, void* buffer, void** bufferData);
	EXPORT int	 btCollisionWorld_ContactPairTest(btCollisionWorld* world, btCollisionObject* bodyA, btCollisionObject* bodyB, int shapeIndexA, int shapeIndexB, void* buffer, void** bufferData);

	EXPORT void* btCollisionWorld_CreateManifoldBuffer();
	EXPORT void  btCollisionWorld_DeleteManifoldBuffer(void* buffer);
	EXPORT int   btCollisionWorld_GetManifolds(btCollisionWorld* world, void* buffer, void** bufferData);
	EXPORT void  btCollisionWorld_GetManifoldContact(btCollisionWorld* world, int manifoldId, int contactId, void* contact);
	EXPORT int   btCollisionWorld_GetManifoldContacts(btCollisionWorld* world, int manifoldId, void* contactBuffer, int contactBufferSize);

	EXPORT void* btCollisionWorld_CreateRayDataBuffer();
	EXPORT void  btCollisionWorld_DeleteRayDataBuffer(void* buffer);	
	EXPORT int   btCollisionWorld_RayTest(btCollisionWorld* world, btVector3* rayFromWorld, btVector3* rayToWorld, int filterMask, void* buffer, void** bufferData);
	EXPORT int   btCollisionWorld_RayTestAll(btCollisionWorld* world, btVector3* rayFromWorld, btVector3* rayToWorld, int filterMask, void* buffer, void** bufferData);

	EXPORT int   btCollisionWorld_SweepTest(btCollisionWorld* world, btConvexShape* castShape, btCollisionObject* castOwner, btTransform* rayFromWorld, btTransform* rayToWorld, int filterMask, void* buffer, void** bufferData);
	EXPORT int   btCollisionWorld_SweepTestAll(btCollisionWorld* world, btConvexShape* castShape, btCollisionObject* castOwner, btTransform* rayFromWorld, btTransform* rayToWorld, int filterMask, void* buffer, void** bufferData);

	// Evergine debug draw...
	EXPORT btEvergineDebugDrawWrapper* btEvergineDebugDrawWrapper_new(void* debugDrawGCHandle);
	EXPORT void* btEvergineDebugDrawWrapper_getGCHandle(btEvergineDebugDrawWrapper* obj);

	EXPORT void btEvergineDebugDrawWrapper_reset(btEvergineDebugDrawWrapper* obj);
	EXPORT void btEvergineDebugDrawWrapper_getDrawInformation(btEvergineDebugDrawWrapper* obj, int* commandCount, void** buffer);
	EXPORT int btEvergineDebugDrawWrapper_getDebugMode(btEvergineDebugDrawWrapper* obj);
	EXPORT void btEvergineDebugDrawWrapper_setDebugMode(btEvergineDebugDrawWrapper* obj, int debugMode);
	EXPORT void btEvergineDebugDrawWrapper_setDefaultColors(btEvergineDebugDrawWrapper* obj, void* defaultColors);

	EXPORT void btEvergineDebugDrawWrapper_delete(btEvergineDebugDrawWrapper* obj);


	// Add methods to match WASM cookies...
	EXPORT btRigidBody_btRigidBodyConstructionInfo* btRigidBody_btRigidBodyConstructionInfo_new_fixCookie(btScalar mass, btMotionState* motionState, btCollisionShape* collisionShape, const btTransform* transform);
	EXPORT int btDynamicsWorld_stepSimulation_fixCookie(btDynamicsWorld* obj, btScalar timeStep, int maxSubSteps, btScalar fixedTimeStep, int i1, int i2, int i3, int i4, int i5);

	//VIIIIIF -> VIIIFII
	EXPORT void btCollisionWorld_convexSweepTest_fixCookie(btCollisionWorld* obj, const btConvexShape* castShape, const btTransform* from, btScalar allowedCcdPenetration, const btTransform* to, btCollisionWorld_ConvexResultCallback* resultCallback);

	EXPORT btIDebugDrawWrapper* btIDebugDrawWrapper_new_fixCookie(void* debugDrawGCHandle, void* callbackPointers);
	
#ifdef __cplusplus
}
#endif
