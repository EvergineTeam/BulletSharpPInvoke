
#include <BulletCollision/CollisionDispatch/btCollisionConfiguration.h>
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btConvexShape.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <LinearMath/btIDebugDraw.h>
#include <LinearMath/btSerializer.h>


#include "conversion.h"
#include "btEvergineContact_wrap.h"

#include <vector>

#define MAX_CONTACTS_PER_OBJECT 1024

#pragma region Contact test
struct ContactData
{
	const void* ColliderA;
	const void* ColliderB;

	int PartIdA;
	int PartIdB;

	int IndexA;
	int IndexB;

	float Distance;

	float NormalX;
	float NormalY;
	float NormalZ;

	float PositionOnAx;
	float PositionOnAy;
	float PositionOnAz;

	float PositionOnBx;
	float PositionOnBy;
	float PositionOnBz;
};

struct ManifoldData
{
	int Id;
	const void* BodyA;
	const void* BodyB;
	int NumContacts;
};

class HasContactCallback : public btCollisionWorld::ContactResultCallback
{
public:
	int NumContacts;

	HasContactCallback(btCollisionObject* bodyA, int shapeIndexA, int shapeIndexB)
		: bodyA(bodyA), NumContacts(0), shapeIndexA(shapeIndexA), shapeIndexB(shapeIndexB)
	{
	}

	virtual	btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0, const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1)
	{
		if (this->bodyA != colObj0Wrap->getCollisionObject()) {
			// Swap index.
			int aux = index0;
			index0 = index1;
			index1 = aux;
		}

		if ((this->shapeIndexA < 0 || index0 == this->shapeIndexA)
			|| (this->shapeIndexB < 0 || index1 == this->shapeIndexB))
		{
			this->NumContacts++;
		}

		return 0;
	}

private:
	btCollisionObject* bodyA;
	int shapeIndexA;
	int shapeIndexB;
};

class ContactCallback : public btCollisionWorld::ContactResultCallback
{
public:
	ContactCallback(btCollisionObject* bodyA, std::vector<ContactData>* buffer, int shapeIndexA, int shapeIndexB)
		: bodyA(bodyA), data(buffer), shapeIndexA(shapeIndexA), shapeIndexB(shapeIndexB)
	{
	}

	virtual	btScalar addSingleResult(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj0Wrap, int partId0, int index0, const btCollisionObjectWrapper* colObj1Wrap, int partId1, int index1)
	{
		auto swap = this->bodyA != colObj0Wrap->getCollisionObject();
		auto objA = swap ? colObj1Wrap : colObj0Wrap;
		auto objB = swap ? colObj0Wrap : colObj1Wrap;

		if (data->size() == MAX_CONTACTS_PER_OBJECT
			|| (this->shapeIndexA >= 0 && objA->m_index != this->shapeIndexA)
			|| (this->shapeIndexB >= 0 && objB->m_index != this->shapeIndexB))
		{
			return 0;
		}

		ContactData result;
		result.ColliderA = objA->getCollisionObject();
		result.ColliderB = objB->getCollisionObject();
		result.PartIdA = objA->m_partId;
		result.PartIdB = objB->m_partId;
		result.IndexA = objA->m_index;
		result.IndexB = objB->m_index;
		result.Distance = cp.getDistance();

		btVector3* posOnA = swap ? &cp.m_positionWorldOnB : &cp.m_positionWorldOnA;
		btVector3* posOnB = swap ? &cp.m_positionWorldOnA : &cp.m_positionWorldOnB;
		float factor = swap ? -1 : 1;

		result.NormalX = factor * cp.m_normalWorldOnB.getX();
		result.NormalY = factor * cp.m_normalWorldOnB.getY();
		result.NormalZ = factor * cp.m_normalWorldOnB.getZ();
		result.PositionOnAx = posOnA->getX();
		result.PositionOnAy = posOnA->getY();
		result.PositionOnAz = posOnA->getZ();
		result.PositionOnBx = posOnB->getX();
		result.PositionOnBy = posOnB->getY();
		result.PositionOnBz = posOnB->getZ();
		data->push_back(result);

		return 0;
	}

private:
	btCollisionObject* bodyA;
	std::vector<ContactData>* data;
	int shapeIndexA;
	int shapeIndexB;
};

int btCollisionWorld_ContactTest(btCollisionWorld* world, btCollisionObject* collisionObject, int shapeIndex, void* buffer, void** bufferData)
{
	std::vector<ContactData>* data = (std::vector<ContactData>*)buffer;

	data->clear();

	ContactCallback cb(collisionObject, data, shapeIndex, -1);
	world->contactTest(collisionObject, cb);

	*bufferData = data->size() ? &(*data)[0] : (void*)0;
	return data->size();
}

int btCollisionWorld_ContactPairTest(btCollisionWorld* world, btCollisionObject* bodyA, btCollisionObject* bodyB, int shapeIndexA, int shapeIndexB, void* buffer, void** bufferData)
{
	if (buffer) 
	{
		std::vector<ContactData>* data = (std::vector<ContactData>*)buffer;
		data->clear();
		ContactCallback cb(bodyA, data, shapeIndexA, shapeIndexB);
		world->contactPairTest(bodyA, bodyB, cb);

		*bufferData = data->size() ? &(*data)[0] : (void*)0;
		return data->size();
	}
	else {
		HasContactCallback cb(bodyA, shapeIndexA, shapeIndexB);
		world->contactPairTest(bodyA, bodyB, cb);

		return cb.NumContacts;
	}
}

void* btCollisionWorld_CreateBuffer()
{
	std::vector<ContactData>* buffer = new std::vector<ContactData>(MAX_CONTACTS_PER_OBJECT);
	return buffer;
}

void btCollisionWorld_DeleteBuffer(void* buffer)
{
	std::vector<ContactData>* data = (std::vector<ContactData>*)buffer;
	delete data;
}

int btCollisionWorld_GetManifolds(btCollisionWorld* world, void* buffer, void** bufferData)
{
	std::vector<ManifoldData>* data = (std::vector<ManifoldData>*)buffer;
	data->clear();
	btDispatcher* dispatcher = world->getDispatcher();
	int numManifolds = dispatcher->getNumManifolds();

	for (int i = 0; i < numManifolds; i++)
	{
		btPersistentManifold* manifold = dispatcher->getManifoldByIndexInternal(i);

		ManifoldData manifoldData;
		manifoldData.Id = i;
		manifoldData.BodyA = manifold->getBody0();
		manifoldData.BodyB = manifold->getBody1();
		manifoldData.NumContacts = manifold->getNumContacts();
		data->push_back(manifoldData);
	}

	*bufferData = data->size() ? &(*data)[0] : (void*)0;

	return numManifolds;
}

void btCollisionWorld_GetManifoldContact(btCollisionWorld* world, int manifoldId, int contactId, void* ptr)
{
	btDispatcher* dispatcher = world->getDispatcher();
	btPersistentManifold* manifold = dispatcher->getManifoldByIndexInternal(manifoldId);

	const btCollisionObject* bodyA = manifold->getBody0();
	const btCollisionObject* bodyB = manifold->getBody1();

	btManifoldPoint& cp = manifold->getContactPoint(contactId);

	ContactData* contact = (ContactData*)ptr;
	contact->ColliderA = bodyA;
	contact->ColliderB = bodyB;
	contact->PartIdA = cp.m_partId0;
	contact->PartIdB = cp.m_partId1;
	contact->IndexA = cp.m_index0;
	contact->IndexB = cp.m_index1;
	contact->Distance = cp.getDistance();
	contact->NormalX = cp.m_normalWorldOnB.getX();
	contact->NormalY = cp.m_normalWorldOnB.getY();
	contact->NormalZ = cp.m_normalWorldOnB.getZ();
	contact->PositionOnAx = cp.m_positionWorldOnA.getX();
	contact->PositionOnAy = cp.m_positionWorldOnA.getY();
	contact->PositionOnAz = cp.m_positionWorldOnA.getZ();
	contact->PositionOnBx = cp.m_positionWorldOnB.getX();
	contact->PositionOnBy = cp.m_positionWorldOnB.getY();
	contact->PositionOnBz = cp.m_positionWorldOnB.getZ();
}

int btCollisionWorld_GetManifoldContacts(btCollisionWorld* world, int manifoldId, void* bufferPtr, int contactBufferSize)
{
	ContactData* contactBuffer = (ContactData*)bufferPtr;
	btDispatcher* dispatcher = world->getDispatcher();	
	btPersistentManifold* manifold = dispatcher->getManifoldByIndexInternal(manifoldId);
	int numContacts = manifold->getNumContacts();
	
	if (numContacts > 0)
	{
		if (numContacts > contactBufferSize) numContacts = contactBufferSize;

		const btCollisionObject* bodyA = manifold->getBody0();
		const btCollisionObject* bodyB = manifold->getBody1();

		for (int c = 0; c < numContacts; c++)
		{
			btManifoldPoint& cp = manifold->getContactPoint(c);

			ContactData* contactData = &contactBuffer[c];
			contactData->ColliderA = bodyA;
			contactData->ColliderB = bodyB;
			contactData->PartIdA = cp.m_partId0;
			contactData->PartIdB = cp.m_partId1;
			contactData->IndexA = cp.m_index0;
			contactData->IndexB = cp.m_index1;
			contactData->Distance = cp.getDistance();
			contactData->NormalX = cp.m_normalWorldOnB.getX();
			contactData->NormalY = cp.m_normalWorldOnB.getY();
			contactData->NormalZ = cp.m_normalWorldOnB.getZ();
			contactData->PositionOnAx = cp.m_positionWorldOnA.getX();
			contactData->PositionOnAy = cp.m_positionWorldOnA.getY();
			contactData->PositionOnAz = cp.m_positionWorldOnA.getZ();
			contactData->PositionOnBx = cp.m_positionWorldOnB.getX();
			contactData->PositionOnBy = cp.m_positionWorldOnB.getY();
			contactData->PositionOnBz = cp.m_positionWorldOnB.getZ();
		}
	}

	return numContacts;
}

void* btCollisionWorld_CreateManifoldBuffer()
{
	std::vector<ManifoldData>* buffer = new std::vector<ManifoldData>();
	return buffer;
}

void btCollisionWorld_DeleteManifoldBuffer(void* buffer)
{
	std::vector<ManifoldData>* data = (std::vector<ManifoldData>*)buffer;
	delete data;
}
#pragma endregion

#pragma region RayCast test
struct RayResultData
{
	const void* CollisionObject;

	int ShapePart;

	int TriangleIndex;

	float HitFraction;

	float PointX;
	float PointY;
	float PointZ;

	float NormalX;
	float NormalY;
	float NormalZ;
};

struct	ClosestRayTestCallback : public btCollisionWorld::RayResultCallback
{
	ClosestRayTestCallback(const btVector3&	rayFromWorld, const btVector3&	rayToWorld)
		:m_rayFromWorld(rayFromWorld),
		m_rayToWorld(rayToWorld),
		m_shapePart(0),
		m_triangleIndex(0)
	{
	}

	btVector3	m_rayFromWorld;//used to calculate hitPointWorld from hitFraction
	btVector3	m_rayToWorld;

	btVector3	m_hitNormalWorld;
	btVector3	m_hitPointWorld;

	int m_shapePart;
	int m_triangleIndex;

	virtual	btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace)
	{
		m_closestHitFraction = rayResult.m_hitFraction;

		if (rayResult.m_localShapeInfo) {
			m_triangleIndex = rayResult.m_localShapeInfo->m_triangleIndex;
			m_shapePart = rayResult.m_localShapeInfo->m_shapePart;
		}
		else {
			m_triangleIndex = 0;
			m_shapePart = 0;
		}
		
		m_collisionObject = rayResult.m_collisionObject;
		if (normalInWorldSpace)
		{
			m_hitNormalWorld = rayResult.m_hitNormalLocal;
		}
		else
		{
			///need to transform normal into worldspace
			m_hitNormalWorld = m_collisionObject->getWorldTransform().getBasis()*rayResult.m_hitNormalLocal;
		}

		m_hitPointWorld.setInterpolate3(m_rayFromWorld, m_rayToWorld, rayResult.m_hitFraction);
		return rayResult.m_hitFraction;
	}
};

struct	RaycastCallback : public btCollisionWorld::RayResultCallback
{
	RaycastCallback(std::vector<RayResultData>* buffer, const btVector3& rayFromWorld, const btVector3&	rayToWorld)
		: m_data(buffer),
		m_rayFromWorld(rayFromWorld),
		m_rayToWorld(rayToWorld)
	{
	}

	btVector3	m_rayFromWorld;//used to calculate hitPointWorld from hitFraction
	btVector3	m_rayToWorld;

	virtual	btScalar	addSingleResult(btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace)
	{
		RayResultData result;

		if (rayResult.m_localShapeInfo) {
			result.TriangleIndex = rayResult.m_localShapeInfo->m_triangleIndex;
			result.ShapePart = rayResult.m_localShapeInfo->m_shapePart;
		}
		else {
			result.TriangleIndex = 0;
			result.ShapePart = 0;
		}

		result.CollisionObject = rayResult.m_collisionObject;
		result.HitFraction = rayResult.m_hitFraction;

		btVector3 normal, point;
		if (normalInWorldSpace)
		{
			normal = rayResult.m_hitNormalLocal;
		}
		else
		{
			///need to transform normal into worldspace
			normal = rayResult.m_collisionObject->getWorldTransform().getBasis()*rayResult.m_hitNormalLocal;
		}

		point.setInterpolate3(m_rayFromWorld, m_rayToWorld, rayResult.m_hitFraction);

		result.NormalX = normal.getX();
		result.NormalY = normal.getY();
		result.NormalZ = normal.getZ();

		result.PointX = point.getX();
		result.PointY = point.getY();
		result.PointZ = point.getZ();

		m_data->push_back(result);

		return m_closestHitFraction;
	}
private:
	std::vector<RayResultData>* m_data;
};

int btCollisionWorld_RayTest(btCollisionWorld* world, btVector3* rayFromWorld, btVector3* rayToWorld, int filterMask, void* buffer, void** bufferData)
{
	BTVECTOR3_IN(rayFromWorld);
	BTVECTOR3_IN(rayToWorld);

	ClosestRayTestCallback cb(BTVECTOR3_USE(rayFromWorld), BTVECTOR3_USE(rayToWorld));
	cb.m_collisionFilterGroup = btBroadphaseProxy::AllFilter;
	cb.m_collisionFilterMask = filterMask;

	world->rayTest(BTVECTOR3_USE(rayFromWorld), BTVECTOR3_USE(rayToWorld), cb);	

	bool success = cb.hasHit();

	std::vector<RayResultData>* data = (std::vector<RayResultData>*)buffer;
	data->clear();

	if (success) {
		RayResultData result;
		result.CollisionObject = cb.m_collisionObject;

		int shapeIndex = cb.m_shapePart - 1;
		if (shapeIndex < 0) {
			shapeIndex = cb.m_triangleIndex;
		}

		result.HitFraction = cb.m_closestHitFraction;
		
		result.PointX = cb.m_hitPointWorld.getX();
		result.PointY = cb.m_hitPointWorld.getY();
		result.PointZ = cb.m_hitPointWorld.getZ();

		result.NormalX = cb.m_hitNormalWorld.getX();
		result.NormalY = cb.m_hitNormalWorld.getY();
		result.NormalZ = cb.m_hitNormalWorld.getZ();

		result.ShapePart = cb.m_shapePart;
		result.TriangleIndex = cb.m_triangleIndex;

		data->push_back(result);
	}

	*bufferData = data->size() ? &(*data)[0] : (void*)0;

	return data->size();
}

int btCollisionWorld_RayTestAll(btCollisionWorld* world, btVector3* rayFromWorld, btVector3* rayToWorld, int filterMask, void* buffer, void** bufferData)
{
	BTVECTOR3_IN(rayFromWorld);
	BTVECTOR3_IN(rayToWorld);
	std::vector<RayResultData>* data = (std::vector<RayResultData>*)buffer;

	RaycastCallback cb(data, BTVECTOR3_USE(rayFromWorld), BTVECTOR3_USE(rayToWorld));
	cb.m_collisionFilterGroup = btBroadphaseProxy::AllFilter;
	cb.m_collisionFilterMask = filterMask;

	data->clear();
	world->rayTest(BTVECTOR3_USE(rayFromWorld), BTVECTOR3_USE(rayToWorld), cb);

	*bufferData = data->size() ? &(*data)[0] : (void*)0;
	return data->size();
}

void* btCollisionWorld_CreateRayDataBuffer()
{
	std::vector<RayResultData>* data = new std::vector<RayResultData>(MAX_CONTACTS_PER_OBJECT);
	return data;
}

void btCollisionWorld_DeleteRayDataBuffer(void* buffer)
{
	std::vector<RayResultData>* data = (std::vector<RayResultData>*)buffer;
	delete data;
}
#pragma endregion

#pragma region Sweep test

struct	ClosestConvexTestCallback : public btCollisionWorld::ConvexResultCallback
{
	ClosestConvexTestCallback(btCollisionObject* castOwner)
		: m_shapePart(0),
		m_triangleIndex(0),
		m_castOwner(castOwner)
	{
	}

	btVector3	m_hitNormalWorld;
	btVector3	m_hitPointWorld;
	const btCollisionObject*	m_hitCollisionObject;
	const btCollisionObject* m_castOwner;

	int m_shapePart;
	int m_triangleIndex;

	virtual	btScalar addSingleResult(btCollisionWorld::LocalConvexResult& rayResult, bool normalInWorldSpace)
	{
		if (m_castOwner == rayResult.m_hitCollisionObject)
		{
			return m_closestHitFraction;
		}

		m_hitCollisionObject = rayResult.m_hitCollisionObject;
		m_closestHitFraction = rayResult.m_hitFraction;

		if (rayResult.m_localShapeInfo) {
			m_triangleIndex = rayResult.m_localShapeInfo->m_triangleIndex;
			m_shapePart = rayResult.m_localShapeInfo->m_shapePart;
		}
		else {
			m_triangleIndex = 0;
			m_shapePart = 0;
		}

		if (normalInWorldSpace)
		{
			m_hitNormalWorld = rayResult.m_hitNormalLocal;
		}
		else
		{
			///need to transform normal into worldspace
			m_hitNormalWorld = m_hitCollisionObject->getWorldTransform().getBasis()*rayResult.m_hitNormalLocal;
		}

		m_hitPointWorld = rayResult.m_hitPointLocal;
		return rayResult.m_hitFraction;
	}
};

struct	SweepTestCallback : public btCollisionWorld::ConvexResultCallback
{
	SweepTestCallback(btCollisionObject* castOwner, std::vector<RayResultData>* buffer)
		: m_data(buffer),
		m_castOwner(castOwner)
	{
	}

	const btCollisionObject* m_castOwner;

	virtual	btScalar addSingleResult(btCollisionWorld::LocalConvexResult& convexResult, bool normalInWorldSpace)
	{
		if (m_castOwner == convexResult.m_hitCollisionObject)
		{
			return m_closestHitFraction;
		}

		RayResultData result;

		if (convexResult.m_localShapeInfo) 
		{
			result.TriangleIndex = convexResult.m_localShapeInfo->m_triangleIndex;
			result.ShapePart = convexResult.m_localShapeInfo->m_shapePart;
		}
		else 
		{
			result.TriangleIndex = 0;
			result.ShapePart = 0;
		}

		result.CollisionObject = convexResult.m_hitCollisionObject;
		result.HitFraction = convexResult.m_hitFraction;

		btVector3 normal, point;
		if (normalInWorldSpace)
		{
			normal = convexResult.m_hitNormalLocal;
		}
		else
		{
			///need to transform normal into worldspace
			normal = convexResult.m_hitCollisionObject->getWorldTransform().getBasis()*convexResult.m_hitNormalLocal;
		}

		result.NormalX = normal.getX();
		result.NormalY = normal.getY();
		result.NormalZ = normal.getZ();

		result.PointX = convexResult.m_hitPointLocal.getX();
		result.PointY = convexResult.m_hitPointLocal.getY();
		result.PointZ = convexResult.m_hitPointLocal.getZ();

		m_data->push_back(result);

		return m_closestHitFraction;
	}
private:
	std::vector<RayResultData>* m_data;
};

int btCollisionWorld_SweepTest(btCollisionWorld* world, btConvexShape* castShape, btCollisionObject* castOwner, btTransform* rayFromWorld, btTransform* rayToWorld, int filterMask, void* buffer, void** bufferData)
{
	BTTRANSFORM_IN(rayFromWorld);
	BTTRANSFORM_IN(rayToWorld);	
	
	ClosestConvexTestCallback cb(castOwner);
	cb.m_collisionFilterGroup = btBroadphaseProxy::AllFilter;
	cb.m_collisionFilterMask = filterMask;
	world->convexSweepTest(castShape, BTTRANSFORM_USE(rayFromWorld), BTTRANSFORM_USE(rayToWorld), cb);

	bool success = cb.hasHit();

	std::vector<RayResultData>* data = (std::vector<RayResultData>*)buffer;
	data->clear();

	if (success) {
		RayResultData result;
		result.CollisionObject = cb.m_hitCollisionObject;
		result.HitFraction = cb.m_closestHitFraction;

		result.PointX = cb.m_hitPointWorld.getX();
		result.PointY = cb.m_hitPointWorld.getY();
		result.PointZ = cb.m_hitPointWorld.getZ();

		result.NormalX = cb.m_hitNormalWorld.getX();
		result.NormalY = cb.m_hitNormalWorld.getY();
		result.NormalZ = cb.m_hitNormalWorld.getZ();

		result.ShapePart = cb.m_shapePart;
		result.TriangleIndex = cb.m_triangleIndex;

		data->push_back(result);
	}

	*bufferData = data->size() ? &(*data)[0] : (void*)0;
	return data->size();
}

int btCollisionWorld_SweepTestAll(btCollisionWorld* world, btConvexShape* castShape, btCollisionObject* castOwner, btTransform* rayFromWorld, btTransform* rayToWorld, int filterMask, void* buffer, void** bufferData)
{
	BTTRANSFORM_IN(rayFromWorld);
	BTTRANSFORM_IN(rayToWorld);
	std::vector<RayResultData>* data = (std::vector<RayResultData>*)buffer;
	data->clear();

	SweepTestCallback cb(castOwner, data);
	cb.m_collisionFilterGroup = btBroadphaseProxy::DefaultFilter;
	cb.m_collisionFilterMask = filterMask;
	
	world->convexSweepTest(castShape, BTTRANSFORM_USE(rayFromWorld), BTTRANSFORM_USE(rayToWorld), cb);

	*bufferData = data->size() ? &(*data)[0] : (void*)0;
	return data->size();
}
#pragma endregion

#pragma region Missing Cookies

btRigidBody_btRigidBodyConstructionInfo* btRigidBody_btRigidBodyConstructionInfo_new_fixCookie(
	btScalar mass, btMotionState* motionState, btCollisionShape* collisionShape, const btTransform* transform)
{
	return  ALIGNED_NEW(btRigidBody::btRigidBodyConstructionInfo)(mass, motionState, collisionShape);
}

int btDynamicsWorld_stepSimulation_fixCookie(btDynamicsWorld* obj, btScalar timeStep, int maxSubSteps, btScalar fixedTimeStep, int i1, int i2, int i3, int i4, int i5)
{
	return obj->stepSimulation(timeStep, maxSubSteps, fixedTimeStep);
}

#pragma endregion