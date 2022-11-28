
#include <BulletCollision/CollisionDispatch/btCollisionConfiguration.h>
#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btConvexShape.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <LinearMath/btIDebugDraw.h>
#include <LinearMath/btSerializer.h>
#include "btIDebugDraw_wrap.h"

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
	ClosestRayTestCallback(const btVector3& rayFromWorld, const btVector3& rayToWorld)
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
			m_hitNormalWorld = m_collisionObject->getWorldTransform().getBasis() * rayResult.m_hitNormalLocal;
		}

		m_hitPointWorld.setInterpolate3(m_rayFromWorld, m_rayToWorld, rayResult.m_hitFraction);
		return rayResult.m_hitFraction;
	}
};

struct	RaycastCallback : public btCollisionWorld::RayResultCallback
{
	RaycastCallback(std::vector<RayResultData>* buffer, const btVector3& rayFromWorld, const btVector3& rayToWorld)
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
			normal = rayResult.m_collisionObject->getWorldTransform().getBasis() * rayResult.m_hitNormalLocal;
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
	const btCollisionObject* m_hitCollisionObject;
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
			m_hitNormalWorld = m_hitCollisionObject->getWorldTransform().getBasis() * rayResult.m_hitNormalLocal;
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
			normal = convexResult.m_hitCollisionObject->getWorldTransform().getBasis() * convexResult.m_hitNormalLocal;
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

#pragma region Debug Draw

enum DrawCommandType {
	DrawAABB = 0,
	DrawArc,
	DrawBox,
	DrawCapsule,
	DrawCone,
	DrawContactPoint,
	DrawCylinder,
	DrawLine,
	DrawPlane,
	DrawSphere,
	DrawSpherePatch,
	DrawTransform,
	DrawTriangle
};

struct unalignedVector3 {
	float x;
	float y;
	float z;
};

struct unalignedTransform {
	float m[16];
};

struct command_DrawAAbb {
	unalignedVector3 from;
	unalignedVector3 to;
	unalignedVector3 color;
};

struct command_DrawArc {
	unalignedVector3 center;
	unalignedVector3 normal;
	unalignedVector3 axis;
	float radiusA;
	float radiusB;
	float minAngle;
	float maxAngle;
	unalignedVector3 color;
	bool drawSect;
	float stepDegrees;
};

struct command_DrawBox {
	unalignedVector3 bbMin;
	unalignedVector3 bbMax;
	unalignedTransform trans;
	unalignedVector3 color;
};

struct command_DrawCapsule {
	float radius;
	float halfHeight;
	int upAxis;
	unalignedTransform trans;
	unalignedVector3 color;
};

struct command_DrawCone {
	float radius;
	float height;
	int upAxis;
	unalignedTransform trans;
	unalignedVector3 color;
};

struct command_DrawContactPoint {
	unalignedVector3 PointOnB;
	unalignedVector3 normalOnB;
	float distance;
	int lifeTime;
	unalignedVector3 color;
};

struct command_DrawCylinder {
	float radius;
	float halfHeight;
	int upAxis;
	unalignedTransform trans;
	unalignedVector3 color;
};

struct command_DrawLine {
	unalignedVector3 from;
	unalignedVector3 to;
	unalignedVector3 color;
};

struct command_DrawPlane {
	unalignedVector3 planeNormal;
	float planeConst;
	unalignedTransform trans;
	unalignedVector3 color;
};

struct command_DrawSphere {
	float radius;
	unalignedTransform trans;
	unalignedVector3 color;
};

struct command_DrawSpherePatch {
	unalignedVector3 center;
	unalignedVector3 up;
	unalignedVector3 axis;
	float radius;
	float minTh;
	float maxTh;
	float minPs;
	float maxPs;
	unalignedVector3 color;
	float stepDegrees;
};

struct command_DrawTransform {
	unalignedTransform trans;
	float orthoLen;
};

struct command_DrawTriangle {
	unalignedVector3 v0;
	unalignedVector3 v1;
	unalignedVector3 v2;
	unalignedVector3 color;
	float alpha;
};

#define ASSIGN_VECTOR3_UNALIGNED(v) *(unalignedVector3*)&BTVECTOR3_USE(v)
#define ASSIGN_TRANSFORM_UNALIGNED(v) *(unalignedTransform*)&BTTRANSFORM_USE_REF(v)

#define ADD_TO_DRAWBUFFER(ptr, cmdType, commandType, command) \
	ensureCapacity(&ptr, sizeof(cmdType) + 4); \
	\
	*(int*)ptr = (int)commandType; \
	ptr = static_cast<int*>(ptr) + 1; \
	\
	*(cmdType*)ptr = command; \
	ptr = static_cast<cmdType*>(ptr) + 1; \
	this->drawCommandCount++;

int NextPowerOfTwo(int v)
{
	v--;
	v |= v >> 1;
	v |= v >> 2;
	v |= v >> 4;
	v |= v >> 8;
	v |= v >> 16;
	v++;

	return v;
}

void btEvergineDebugDrawWrapper::ensureCapacity(void** ptr, int size)
{
	int newUsage = this->currentUsage + size;

	if (!this->buffer || newUsage > this->bufferCapacity)
	{
		int nextCapacity = NextPowerOfTwo(newUsage);
		void* nextBuffer = malloc(nextCapacity);

		if (this->buffer) 
		{
			// Copy the old buffer to the new buffer...
			memcpy(nextBuffer, this->buffer, this->bufferCapacity);
			free(this->buffer);
		}

		this->buffer = nextBuffer;
		this->bufferCapacity = nextCapacity;
		*ptr = (void*)((size_t)this->buffer + this->currentUsage);
	}

	this->currentUsage = newUsage;
}

btEvergineDebugDrawWrapper::btEvergineDebugDrawWrapper(void* debugDrawGCHandle)
{
	this->_debugDrawGCHandle = debugDrawGCHandle;
	this->drawCommandCount = 0;
	this->bufferCapacity = 1024; // 1k by default...
	this->buffer = malloc(this->bufferCapacity);
}

btEvergineDebugDrawWrapper::~btEvergineDebugDrawWrapper()
{
	free(this->buffer);
	this->buffer = 0;
}

void btEvergineDebugDrawWrapper::reset() 
{
	this->currentPointer = this->buffer;
	this->drawCommandCount = 0;
	this->currentUsage = 0;
}

void btEvergineDebugDrawWrapper::getDrawInformation(int* commandCount, void** buffer) 
{
	*commandCount = this->drawCommandCount;
	*buffer = this->buffer;
}

void  btEvergineDebugDrawWrapper::draw3dText(const btVector3& location, const char* textString)
{
}

void  btEvergineDebugDrawWrapper::drawAabb(const btVector3& from, const btVector3& to, const btVector3& color)
{
	BTVECTOR3_IN(from);
	BTVECTOR3_IN(to);	
	BTVECTOR3_IN(color);

	command_DrawAAbb drawAabbCmd;

	drawAabbCmd.from = ASSIGN_VECTOR3_UNALIGNED(from);
	drawAabbCmd.to = ASSIGN_VECTOR3_UNALIGNED(to);
	drawAabbCmd.color = ASSIGN_VECTOR3_UNALIGNED(color);

	ADD_TO_DRAWBUFFER(this->currentPointer, command_DrawAAbb, DrawAABB, drawAabbCmd);

}

void  btEvergineDebugDrawWrapper::drawArc(const btVector3& center, const btVector3& normal,
	const btVector3& axis, btScalar radiusA, btScalar radiusB, btScalar minAngle, btScalar maxAngle,
	const btVector3& color, bool drawSect, btScalar stepDegrees)
{
	BTVECTOR3_IN(center);
	BTVECTOR3_IN(normal);
	BTVECTOR3_IN(axis);
	BTVECTOR3_IN(color);

	command_DrawArc drawCmd;

	drawCmd.center = ASSIGN_VECTOR3_UNALIGNED(center);
	drawCmd.normal = ASSIGN_VECTOR3_UNALIGNED(normal);
	drawCmd.axis = ASSIGN_VECTOR3_UNALIGNED(axis);
	drawCmd.radiusA = radiusA;
	drawCmd.radiusB = radiusB;
	drawCmd.minAngle = minAngle;
	drawCmd.maxAngle = maxAngle;
	drawCmd.color = ASSIGN_VECTOR3_UNALIGNED(color);
	drawCmd.drawSect = drawSect;
	drawCmd.stepDegrees = stepDegrees;

	ADD_TO_DRAWBUFFER(this->currentPointer, command_DrawArc, DrawArc, drawCmd)
}

void  btEvergineDebugDrawWrapper::drawArc(const btVector3& center, const btVector3& normal, const btVector3& axis,
	btScalar radiusA, btScalar radiusB, btScalar minAngle, btScalar maxAngle,
	const btVector3& color, bool drawSect)
{
}

void  btEvergineDebugDrawWrapper::drawBox(const btVector3& bbMin, const btVector3& bbMax,
	const btVector3& color)
{
}

void  btEvergineDebugDrawWrapper::drawBox(const btVector3& bbMin, const btVector3& bbMax,
	const btTransform& trans, const btVector3& color)
{
	BTVECTOR3_IN(bbMin);
	BTVECTOR3_IN(bbMax);
	BTTRANSFORM_IN_REF(trans);
	BTVECTOR3_IN(color);

	command_DrawBox drawBoxCmd;

	drawBoxCmd.bbMin = ASSIGN_VECTOR3_UNALIGNED(bbMin);
	drawBoxCmd.bbMax = ASSIGN_VECTOR3_UNALIGNED(bbMax);
	drawBoxCmd.trans = ASSIGN_TRANSFORM_UNALIGNED(trans);
	drawBoxCmd.color = ASSIGN_VECTOR3_UNALIGNED(color);

	ADD_TO_DRAWBUFFER(this->currentPointer, command_DrawBox, DrawBox, drawBoxCmd)
}

void  btEvergineDebugDrawWrapper::drawCapsule(btScalar radius, btScalar halfHeight, int upAxis,
	const btTransform& transform, const btVector3& color)
{
	BTTRANSFORM_IN_REF(transform);
	BTVECTOR3_IN(color);

	command_DrawCapsule drawCmd;

	drawCmd.radius = radius;
	drawCmd.halfHeight = halfHeight;
	drawCmd.upAxis = upAxis;
	drawCmd.trans = ASSIGN_TRANSFORM_UNALIGNED(transform);
	drawCmd.color = ASSIGN_VECTOR3_UNALIGNED(color);

	ADD_TO_DRAWBUFFER(this->currentPointer, command_DrawCapsule, DrawCapsule, drawCmd)
}

void  btEvergineDebugDrawWrapper::drawCone(btScalar radius, btScalar height, int upAxis,
	const btTransform& transform, const btVector3& color)
{
	BTTRANSFORM_IN_REF(transform);
	BTVECTOR3_IN(color);

	command_DrawCone drawCmd;

	drawCmd.radius = radius;
	drawCmd.height = height;
	drawCmd.upAxis = upAxis;
	drawCmd.trans = ASSIGN_TRANSFORM_UNALIGNED(transform);
	drawCmd.color = ASSIGN_VECTOR3_UNALIGNED(color);

	ADD_TO_DRAWBUFFER(this->currentPointer, command_DrawCone, DrawCone, drawCmd)
}

void  btEvergineDebugDrawWrapper::drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB,
	btScalar distance, int lifeTime, const btVector3& color)
{
	BTVECTOR3_IN(PointOnB);
	BTVECTOR3_IN(normalOnB);
	BTVECTOR3_IN(color);

	command_DrawContactPoint drawCmd;

	drawCmd.PointOnB = ASSIGN_VECTOR3_UNALIGNED(PointOnB);
	drawCmd.normalOnB = ASSIGN_VECTOR3_UNALIGNED(normalOnB);
	drawCmd.distance = distance;
	drawCmd.lifeTime = lifeTime;
	drawCmd.color = ASSIGN_VECTOR3_UNALIGNED(color);

	ADD_TO_DRAWBUFFER(this->currentPointer, command_DrawContactPoint, DrawContactPoint, drawCmd);
}

void  btEvergineDebugDrawWrapper::drawCylinder(btScalar radius, btScalar halfHeight, int upAxis,
	const btTransform& transform, const btVector3& color)
{
	BTTRANSFORM_IN_REF(transform);
	BTVECTOR3_IN(color);

	command_DrawCylinder drawCmd;

	drawCmd.radius = radius;
	drawCmd.halfHeight = halfHeight;
	drawCmd.upAxis = upAxis;
	drawCmd.trans = ASSIGN_TRANSFORM_UNALIGNED(transform);
	drawCmd.color = ASSIGN_VECTOR3_UNALIGNED(color);

	ADD_TO_DRAWBUFFER(this->currentPointer, command_DrawCylinder, DrawCylinder, drawCmd)
}

void  btEvergineDebugDrawWrapper::drawLine(const btVector3& from, const btVector3& to, const btVector3& color)
{
	BTVECTOR3_IN(from);
	BTVECTOR3_IN(to);
	BTVECTOR3_IN(color);

	command_DrawLine drawCmd;

	drawCmd.from = ASSIGN_VECTOR3_UNALIGNED(from);
	drawCmd.to = ASSIGN_VECTOR3_UNALIGNED(to);
	drawCmd.color = ASSIGN_VECTOR3_UNALIGNED(color);

	ADD_TO_DRAWBUFFER(this->currentPointer, command_DrawLine, DrawLine, drawCmd);
}

void  btEvergineDebugDrawWrapper::drawPlane(const btVector3& planeNormal, btScalar planeConst,
	const btTransform& transform, const btVector3& color)
{
	BTVECTOR3_IN(planeNormal);
	BTTRANSFORM_IN_REF(transform);
	BTVECTOR3_IN(color);

	command_DrawPlane drawBoxCmd;

	drawBoxCmd.planeNormal = ASSIGN_VECTOR3_UNALIGNED(planeNormal);
	drawBoxCmd.planeConst = planeConst;
	drawBoxCmd.trans = ASSIGN_TRANSFORM_UNALIGNED(transform);
	drawBoxCmd.color = ASSIGN_VECTOR3_UNALIGNED(color);

	ADD_TO_DRAWBUFFER(this->currentPointer, command_DrawPlane, DrawPlane, drawBoxCmd)
}

void  btEvergineDebugDrawWrapper::drawSphere(const btVector3& p, btScalar radius, const btVector3& color)
{
}

void  btEvergineDebugDrawWrapper::drawSphere(btScalar radius, const btTransform& trans,
	const btVector3& color)
{
	BTTRANSFORM_IN_REF(trans);
	BTVECTOR3_IN(color);

	command_DrawSphere drawSphereCmd;
	drawSphereCmd.radius = radius;
	drawSphereCmd.trans = ASSIGN_TRANSFORM_UNALIGNED(trans);
	drawSphereCmd.color = ASSIGN_VECTOR3_UNALIGNED(color);

	ADD_TO_DRAWBUFFER(this->currentPointer, command_DrawSphere, DrawSphere, drawSphereCmd);
}

void  btEvergineDebugDrawWrapper::drawSpherePatch(const btVector3& center, const btVector3& up,
	const btVector3& axis, btScalar radius, btScalar minTh, btScalar maxTh, btScalar minPs,
	btScalar maxPs, const btVector3& color, btScalar stepDegrees)
{
	BTVECTOR3_IN(center);
	BTVECTOR3_IN(up);
	BTVECTOR3_IN(axis);
	BTVECTOR3_IN(color);

	command_DrawSpherePatch drawCmd;

	drawCmd.center = ASSIGN_VECTOR3_UNALIGNED(center);
	drawCmd.up = ASSIGN_VECTOR3_UNALIGNED(up);
	drawCmd.axis = ASSIGN_VECTOR3_UNALIGNED(axis);
	drawCmd.radius = radius;
	drawCmd.minTh = minTh;
	drawCmd.maxTh = maxTh;
	drawCmd.maxPs = maxPs;
	drawCmd.color = ASSIGN_VECTOR3_UNALIGNED(color);
	drawCmd.stepDegrees = stepDegrees;

	ADD_TO_DRAWBUFFER(this->currentPointer, command_DrawSpherePatch, DrawSpherePatch, drawCmd);
}

void  btEvergineDebugDrawWrapper::drawSpherePatch(const btVector3& center, const btVector3& up, const btVector3& axis, btScalar radius,
	btScalar minTh, btScalar maxTh, btScalar minPs, btScalar maxPs, const btVector3& color)
{
}

void  btEvergineDebugDrawWrapper::drawTransform(const btTransform& transform, btScalar orthoLen)
{
	BTTRANSFORM_IN_REF(transform);

	int sizeCmd = sizeof(command_DrawTransform);

	command_DrawTransform drawCmd;
	drawCmd.trans = ASSIGN_TRANSFORM_UNALIGNED(transform);
	drawCmd.orthoLen = orthoLen;

	ADD_TO_DRAWBUFFER(this->currentPointer, command_DrawTransform, DrawTransform, drawCmd);
}

void  btEvergineDebugDrawWrapper::drawTriangle(const btVector3& v0, const btVector3& v1,
	const btVector3& v2, const btVector3& color, btScalar __unnamed4)
{
	BTVECTOR3_IN(v0);
	BTVECTOR3_IN(v1);
	BTVECTOR3_IN(v2);
	BTVECTOR3_IN(color);

	command_DrawTriangle drawCmd;

	drawCmd.v0 = ASSIGN_VECTOR3_UNALIGNED(v0);
	drawCmd.v1 = ASSIGN_VECTOR3_UNALIGNED(v1);
	drawCmd.v2 = ASSIGN_VECTOR3_UNALIGNED(v2);
	drawCmd.color = ASSIGN_VECTOR3_UNALIGNED(color);

	ADD_TO_DRAWBUFFER(this->currentPointer, command_DrawTriangle, DrawTriangle, drawCmd);

}

void  btEvergineDebugDrawWrapper::drawTriangle(const btVector3& v0, const btVector3& v1,
	const btVector3& v2, const btVector3& __unnamed3, const btVector3& __unnamed4,
	const btVector3& __unnamed5, const btVector3& color, btScalar alpha)
{
}

void  btEvergineDebugDrawWrapper::baseDrawAabb(const btVector3& from, const btVector3& to, const btVector3& color)
{
	btIDebugDraw::drawAabb(from, to, color);
}

void  btEvergineDebugDrawWrapper::baseDrawCone(btScalar radius, btScalar height, int upAxis, const btTransform& transform, const btVector3& color)
{
	btIDebugDraw::drawCone(radius, height, upAxis, transform, color);
}

void  btEvergineDebugDrawWrapper::baseDrawCylinder(btScalar radius, btScalar halfHeight, int upAxis, const btTransform& transform, const btVector3& color)
{
	btIDebugDraw::drawCylinder(radius, halfHeight, upAxis, transform, color);
}

void  btEvergineDebugDrawWrapper::baseDrawSphere(const btVector3& p, btScalar radius, const btVector3& color)
{
	btIDebugDraw::drawSphere(p, radius, color);
}

void  btEvergineDebugDrawWrapper::baseDrawTriangle(const btVector3& v0, const btVector3& v1, const btVector3& v2, const btVector3& color, btScalar)
{
	btIDebugDraw::drawTriangle(v0, v1, v2, color, 0);
}

void  btEvergineDebugDrawWrapper::baseDrawTriangle(const btVector3& v0, const btVector3& v1, const btVector3& v2,
	const btVector3& n0, const btVector3& n1, const btVector3& n2, const btVector3& color, btScalar alpha)
{
	btIDebugDraw::drawTriangle(v0, v1, v2, n0, n1, n2, color, alpha);
}

int  btEvergineDebugDrawWrapper::getDebugMode() const
{
	return this->debugDrawModes;
}

void  btEvergineDebugDrawWrapper::reportErrorWarning(const char* warningString)
{
	//_debugDraw->ReportErrorWarning(StringConv::UnmanagedToManaged(warningString));
}

void  btEvergineDebugDrawWrapper::setDebugMode(int debugMode)
{
	this->debugDrawModes = (DebugDrawModes)debugMode;
}

btEvergineDebugDrawWrapper* btEvergineDebugDrawWrapper_new(void* debugDrawGCHandle)
{
	return new btEvergineDebugDrawWrapper(debugDrawGCHandle);
}

void* btEvergineDebugDrawWrapper_getGCHandle(btEvergineDebugDrawWrapper* obj)
{
	return obj->_debugDrawGCHandle;
}

void btEvergineDebugDrawWrapper_reset(btEvergineDebugDrawWrapper* obj)
{
	obj->reset();
}

void btEvergineDebugDrawWrapper_getDrawInformation(btEvergineDebugDrawWrapper* obj, int* commandCount, void** buffer)
{
	obj->getDrawInformation(commandCount, buffer);
}

void btEvergineDebugDrawWrapper_delete(btEvergineDebugDrawWrapper* obj)
{
	delete obj;
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

void btCollisionWorld_convexSweepTest_fixCookie(btCollisionWorld* obj, const btConvexShape* castShape, const btTransform* from, btScalar allowedCcdPenetration, const btTransform* to, btCollisionWorld_ConvexResultCallback* resultCallback)
{
	BTTRANSFORM_IN(from);
	BTTRANSFORM_IN(to);
	obj->convexSweepTest(castShape, BTTRANSFORM_USE(from), BTTRANSFORM_USE(to), *resultCallback,
		allowedCcdPenetration);
}

int btEvergineDebugDrawWrapper_getDebugMode(btEvergineDebugDrawWrapper* obj)
{
	return obj->getDebugMode();
}

void btEvergineDebugDrawWrapper_setDebugMode(btEvergineDebugDrawWrapper* obj, int debugMode)
{
	obj->setDebugMode(debugMode);
}

void btEvergineDebugDrawWrapper_setDefaultColors(btEvergineDebugDrawWrapper* obj, void* defaultColors)
{
	obj->setDefaultColors(*(btIDebugDraw::DefaultColors*)defaultColors);
}

#pragma endregion