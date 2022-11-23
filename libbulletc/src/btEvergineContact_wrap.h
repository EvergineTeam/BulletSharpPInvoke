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

	// Add methods to match WASM cookies...
	EXPORT btRigidBody_btRigidBodyConstructionInfo* btRigidBody_btRigidBodyConstructionInfo_new_fixCookie(btScalar mass, btMotionState* motionState, btCollisionShape* collisionShape, const btTransform* transform);
	EXPORT int btDynamicsWorld_stepSimulation_fixCookie(btDynamicsWorld* obj, btScalar timeStep, int maxSubSteps, btScalar fixedTimeStep, int i1, int i2, int i3, int i4, int i5);
	
#ifdef __cplusplus
}
#endif
