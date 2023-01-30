#include "CollisionDetection.h"
#include "CollisionVolume.h"
#include "AABBVolume.h"
#include "OBBVolume.h"
#include "SphereVolume.h"
#include "Window.h"
#include "Maths.h"
#include "Debug.h"

using namespace NCL;

bool CollisionDetection::RayPlaneIntersection(const Ray&r, const Plane&p, RayCollision& collisions) {
	float ln = Vector3::Dot(p.GetNormal(), r.GetDirection());

	if (ln == 0.0f) {
		return false; //direction vectors are perpendicular!
	}
	
	Vector3 planePoint = p.GetPointOnPlane();

	Vector3 pointDir = planePoint - r.GetPosition();

	float d = Vector3::Dot(pointDir, p.GetNormal()) / ln;

	collisions.collidedAt = r.GetPosition() + (r.GetDirection() * d);

	return true;
}

bool CollisionDetection::RayIntersection(const Ray& r,GameObject& object, RayCollision& collision) {
	bool hasCollided = false;

	const Transform& worldTransform = object.GetTransform();
	const CollisionVolume* volume	= object.GetBoundingVolume();

	if (!volume) {
		return false;
	}

	switch (volume->type) {
		case VolumeType::AABB:		hasCollided = RayAABBIntersection(r, worldTransform, (const AABBVolume&)*volume	, collision); break;
		case VolumeType::OBB:		hasCollided = RayOBBIntersection(r, worldTransform, (const OBBVolume&)*volume	, collision); break;
		case VolumeType::Sphere:	hasCollided = RaySphereIntersection(r, worldTransform, (const SphereVolume&)*volume	, collision); break;

		case VolumeType::Capsule:	hasCollided = RayCapsuleIntersection(r, worldTransform, (const CapsuleVolume&)*volume, collision); break;
	}

	return hasCollided;
}

bool CollisionDetection::RayBoxIntersection(const Ray&r, const Vector3& boxPos, const Vector3& boxSize, RayCollision& collision) {
	Vector3 boxMin = boxPos - boxSize;
	Vector3 boxMax = boxPos + boxSize;
	
	Vector3 rayPos = r.GetPosition();
	Vector3 rayDir = r.GetDirection();

	Vector3 tVals(-1, -1, -1);

	for (int i = 0; i < 3; ++i) { //get best 3 intersections
		if (rayDir[i] > 0) {
			tVals[i] = (boxMin[i] - rayPos[i]) / rayDir[i];
		}
		else if (rayDir[i] < 0) {
			tVals[i] = (boxMax[i] - rayPos[i]) / rayDir[i];
		}
	}
	float bestT = tVals.GetMaxElement();
	if (bestT < 0.0f) {
		return false; //no backwards rays!
	}

	Vector3 intersection = rayPos + (rayDir * bestT);
	const float epsilon = 0.0001f; //an amount of leeway in our calcs
	for (int i = 0; i < 3; ++i) {
		if (intersection[i] + epsilon < boxMin[i] || intersection[i] - epsilon > boxMax[i]) {
			return false; //best intersection doesn�t touch the box!
		}
	}
	collision.collidedAt = intersection;
	collision.rayDistance = bestT;
	return true;
}

bool CollisionDetection::RayAABBIntersection(const Ray&r, const Transform& worldTransform, const AABBVolume& volume, RayCollision& collision) {
	Vector3 boxPos = worldTransform.GetPosition();
	Vector3 boxSize = volume.GetHalfDimensions();
	return RayBoxIntersection(r, boxPos, boxSize, collision);
}

bool CollisionDetection::RayOBBIntersection(const Ray&r, const Transform& worldTransform, const OBBVolume& volume, RayCollision& collision) {
	Quaternion orientation = worldTransform.GetOrientation();
	Vector3 position = worldTransform.GetPosition();
	
	Matrix3 transform = Matrix3(orientation);
	Matrix3 invTransform = Matrix3(orientation.Conjugate());
	
	Vector3 localRayPos = r.GetPosition() - position;
	
	Ray tempRay(invTransform * localRayPos, invTransform * r.GetDirection());

	bool collided = RayBoxIntersection(tempRay, Vector3(), volume.GetHalfDimensions(), collision);

	if (collided) {
		collision.collidedAt = transform * collision.collidedAt + position;
	}
	return collided;
}

bool CollisionDetection::RaySphereIntersection(const Ray&r, const Transform& worldTransform, const SphereVolume& volume, RayCollision& collision) {
	Vector3 spherePos = worldTransform.GetPosition();
	float sphereRadius = volume.GetRadius();

	//Get the direction between the ray origin and the sphere origin
	Vector3 dir = (spherePos - r.GetPosition());

	//Then project the sphere �s origin onto our ray direction vector
	float sphereProj = Vector3::Dot(dir, r.GetDirection());

	if (sphereProj < 0.0f) {
		return false; // point is behind the ray!
	}

	//Get closest point on ray line to sphere
	Vector3 point = r.GetPosition() + (r.GetDirection() * sphereProj);

	float sphereDist = (point - spherePos).Length();
	
	if (sphereDist > sphereRadius) {
		return false;
	}

	float offset = sqrt((sphereRadius * sphereRadius) - (sphereDist * sphereDist));

	collision.rayDistance = sphereProj - (offset);
	collision.collidedAt = r.GetPosition() + (r.GetDirection() * collision.rayDistance);
	return true;
}

bool CollisionDetection::RayCapsuleIntersection(const Ray& r, const Transform& worldTransform, const CapsuleVolume& volume, RayCollision& collision) {
	Vector3 capPos = worldTransform.GetPosition();
	float capRadius = volume.GetRadius();

	//Get the direction between the ray origin and the capsule origin
	Vector3 dir = (capPos - r.GetPosition());

	//Then project the capsule�s origin onto our ray direction vector
	float capProj = Vector3::Dot(dir, r.GetDirection());

	if (capProj < 0.0f) {
		return false; // point is behind the ray!
	}

	//Get closest point on ray line to sphere
	Vector3 point = r.GetPosition() + (r.GetDirection() * capProj);

	Vector3 CapsuleOrientation = worldTransform.GetOrientation() * Vector3(0, volume.GetHalfHeight() - volume.GetRadius(), 0);

	Vector3 CapsuleTop = worldTransform.GetPosition() + CapsuleOrientation;

	Vector3 CapsuleBottom = worldTransform.GetPosition() - CapsuleOrientation;

	Vector3 LinePoint = LineClosestPoint(CapsuleTop, CapsuleBottom, point);

	//Get the direction between the ray origin and the sphere origin
	dir = (LinePoint - r.GetPosition());

	//Then project the sphere�s origin onto our ray direction vector
	float sphereProj = Vector3::Dot(dir, r.GetDirection());

	if (sphereProj < 0.0f) {
		return false; // point is behind the ray!
	}

	//Get closest point on ray line to sphere
	point = r.GetPosition() + (r.GetDirection() * sphereProj);

	float sphereDist = (point - LinePoint).Length();

	if (sphereDist > capRadius) {
		return false;
	}

	float offset = sqrt((capRadius * capRadius) - (sphereDist * sphereDist));

	collision.rayDistance = sphereProj - (offset);
	collision.collidedAt = r.GetPosition() + (r.GetDirection() * collision.rayDistance);
	return true;
}

bool CollisionDetection::ObjectIntersection(GameObject* a, GameObject* b, CollisionInfo& collisionInfo) {
	const CollisionVolume* volA = a->GetBoundingVolume();
	const CollisionVolume* volB = b->GetBoundingVolume();

	if (!volA || !volB) {
		return false;
	}

	collisionInfo.a = a;
	collisionInfo.b = b;

	Transform& transformA = a->GetTransform();
	Transform& transformB = b->GetTransform();

	VolumeType pairType = (VolumeType)((int)volA->type | (int)volB->type);

	//Two AABBs
	if (pairType == VolumeType::AABB) {
		return AABBIntersection((AABBVolume&)*volA, transformA, (AABBVolume&)*volB, transformB, collisionInfo);
	}
	//Two Spheres
	if (pairType == VolumeType::Sphere) {
		return SphereIntersection((SphereVolume&)*volA, transformA, (SphereVolume&)*volB, transformB, collisionInfo);
	}
	//Two OBBs
	if (pairType == VolumeType::OBB) {
		return OBBIntersection((OBBVolume&)*volA, transformA, (OBBVolume&)*volB, transformB, collisionInfo);
	}
	//Two Capsules
	if (pairType == VolumeType::Capsule) {
		return CapsuleIntersection((CapsuleVolume&)*volA, transformA, (CapsuleVolume&)*volB, transformB, collisionInfo);
	}

	//AABB vs Sphere pairs
	if (volA->type == VolumeType::AABB && volB->type == VolumeType::Sphere) {
		return AABBSphereIntersection((AABBVolume&)*volA, transformA, (SphereVolume&)*volB, transformB, collisionInfo);
	}
	if (volA->type == VolumeType::Sphere && volB->type == VolumeType::AABB) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return AABBSphereIntersection((AABBVolume&)*volB, transformB, (SphereVolume&)*volA, transformA, collisionInfo);
	}

	//OBB vs sphere pairs
	if (volA->type == VolumeType::OBB && volB->type == VolumeType::Sphere) {
		return OBBSphereIntersection((OBBVolume&)*volA, transformA, (SphereVolume&)*volB, transformB, collisionInfo);
	}
	if (volA->type == VolumeType::Sphere && volB->type == VolumeType::OBB) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return OBBSphereIntersection((OBBVolume&)*volB, transformB, (SphereVolume&)*volA, transformA, collisionInfo);
	}

	//OBB vs AABB
	//if (volA->type == VolumeType::OBB && volB->type == VolumeType::AABB) {
	//	return OBBAABBIntersection((OBBVolume&)*volA, transformA, (AABBVolume&)*volB, transformB, collisionInfo);
	//}
	//if (volA->type == VolumeType::AABB && volB->type == VolumeType::OBB) {
	//	collisionInfo.a = b;
	//	collisionInfo.b = a;
	//	return OBBAABBIntersection((OBBVolume&)*volB, transformB, (AABBVolume&)*volA, transformA, collisionInfo);
	//}

	//Capsule vs other interactions
	if (volA->type == VolumeType::Capsule && volB->type == VolumeType::Sphere) {
		return SphereCapsuleIntersection((CapsuleVolume&)*volA, transformA, (SphereVolume&)*volB, transformB, collisionInfo);
	}
	if (volA->type == VolumeType::Sphere && volB->type == VolumeType::Capsule) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return SphereCapsuleIntersection((CapsuleVolume&)*volB, transformB, (SphereVolume&)*volA, transformA, collisionInfo);
	}

	if (volA->type == VolumeType::Capsule && volB->type == VolumeType::AABB) {
		return AABBCapsuleIntersection((CapsuleVolume&)*volA, transformA, (AABBVolume&)*volB, transformB, collisionInfo);
	}
	if (volB->type == VolumeType::Capsule && volA->type == VolumeType::AABB) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return AABBCapsuleIntersection((CapsuleVolume&)*volB, transformB, (AABBVolume&)*volA, transformA, collisionInfo);
	}

	if (volA->type == VolumeType::Capsule && volB->type == VolumeType::OBB) {
		return OBBCapsuleIntersection((CapsuleVolume&)*volA, transformA, (OBBVolume&)*volB, transformB, collisionInfo);
	}
	if (volB->type == VolumeType::Capsule && volA->type == VolumeType::OBB) {
		collisionInfo.a = b;
		collisionInfo.b = a;
		return OBBCapsuleIntersection((CapsuleVolume&)*volB, transformB, (OBBVolume&)*volA, transformA, collisionInfo);
	}

	return false;
}

bool CollisionDetection::AABBTest(const Vector3& posA, const Vector3& posB, const Vector3& halfSizeA, const Vector3& halfSizeB) {
	Vector3 delta = posB - posA;
	Vector3 totalSize = halfSizeA + halfSizeB;

	if (abs(delta.x) < totalSize.x &&
		abs(delta.y) < totalSize.y &&
		abs(delta.z) < totalSize.z) {
		return true;
	}
	return false;
}

//AABB/AABB Collisions
bool CollisionDetection::AABBIntersection(const AABBVolume& volumeA, const Transform& worldTransformA,
	const AABBVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {
	Vector3 boxAPos = worldTransformA.GetPosition();
	Vector3 boxBPos = worldTransformB.GetPosition();
	
	Vector3 boxASize = volumeA.GetHalfDimensions();
	Vector3 boxBSize = volumeB.GetHalfDimensions();

	bool overlap = AABBTest(boxAPos, boxBPos, boxASize, boxBSize);

	if (overlap) {
		static const Vector3 faces[6] =
			{
				Vector3(-1, 0, 0), Vector3(1, 0, 0),
				Vector3(0, -1, 0), Vector3(0, 1, 0),
				Vector3(0, 0, -1), Vector3(0, 0, 1),
			};

		Vector3 maxA = boxAPos + boxASize;
		Vector3 minA = boxAPos - boxASize;

		Vector3 maxB = boxBPos + boxBSize;
		Vector3 minB = boxBPos - boxBSize;

		float distances[6] =
		{
			(maxB.x - minA.x),// distance of box �b� to �left� of �a�.
			(maxA.x - minB.x),// distance of box �b� to �right� of �a�.
			(maxB.y - minA.y),// distance of box �b� to �bottom � of �a�.
			(maxA.y - minB.y),// distance of box �b� to �top� of �a�.
			(maxB.z - minA.z),// distance of box �b� to �far� of �a�.
			(maxA.z - minB.z) // distance of box �b� to �near� of �a�.
		};
		float penetration = FLT_MAX;
		Vector3 bestAxis;

		for (int i = 0; i < 6; i++)
		{
			if (distances[i] < penetration) {
				penetration = distances[i];
				bestAxis = faces[i];
			}
		}
		collisionInfo.AddContactPoint(Vector3(), Vector3(), bestAxis, penetration);
		return true;
	}
	return false;
}

//Sphere / Sphere Collision
bool CollisionDetection::SphereIntersection(const SphereVolume& volumeA, const Transform& worldTransformA,
	const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	float radii = volumeA.GetRadius() + volumeB.GetRadius();
	Vector3 delta = worldTransformB.GetPosition() - worldTransformA.GetPosition();

	float deltaLength = delta.Length();

	if (deltaLength < radii) {
		float penetration = (radii - deltaLength);
		Vector3 normal = delta.Normalised();
		Vector3 localA = normal * volumeA.GetRadius();
		Vector3 localB = -normal * volumeB.GetRadius();
		
		collisionInfo.AddContactPoint(localA, localB, normal, penetration);
		return true;//we�re colliding!
	}
	return false;
}

//Capsule / Capsule Collision
bool CollisionDetection::CapsuleIntersection(
	const CapsuleVolume& volumeA, const Transform& worldTransformA,
	const CapsuleVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	float radii = volumeA.GetRadius() + volumeB.GetRadius();

	Vector3 CapsuleAOrientation = worldTransformA.GetOrientation() * Vector3(0, volumeA.GetHalfHeight() - volumeA.GetRadius(), 0);
	Vector3 CapsuleATop = worldTransformA.GetPosition() + CapsuleAOrientation;
	Vector3 CapsuleABottom = worldTransformA.GetPosition() - CapsuleAOrientation;

	Vector3 CapsuleBOrientation = worldTransformB.GetOrientation() * Vector3(0, volumeB.GetHalfHeight() - volumeB.GetRadius(), 0);
	Vector3 CapsuleBTop = worldTransformB.GetPosition() + CapsuleBOrientation;
	Vector3 CapsuleBBottom = worldTransformB.GetPosition() - CapsuleBOrientation;

	Vector3 v0 = CapsuleBBottom - CapsuleABottom;
	Vector3 v1 = CapsuleBTop - CapsuleABottom;
	Vector3 v2 = CapsuleBBottom - CapsuleATop;
	Vector3 v3 = CapsuleBTop - CapsuleATop;

	float d0 = Vector3::Dot(v0, v0);
	float d1 = Vector3::Dot(v1, v1);
	float d2 = Vector3::Dot(v2, v2);
	float d3 = Vector3::Dot(v3, v3);

	Vector3 bestA;
	if (d2 < d0 || d2 < d1 || d3 < d0 || d3 < d1) {
		bestA = CapsuleATop;
	}
	else {
		bestA = CapsuleABottom;
	}

	Vector3 bestB = LineClosestPoint(CapsuleBBottom, CapsuleBTop, bestA);
	bestA = LineClosestPoint(CapsuleABottom, CapsuleATop, bestB);

	Vector3 delta = bestB - bestA;
	float deltaLength = delta.Length();

	if (deltaLength < radii) {
		float penetration = (radii - deltaLength);
		Vector3 normal = delta.Normalised();
		Vector3 localA = (bestA - worldTransformA.GetPosition()) + (normal * volumeA.GetRadius());
		Vector3 localB = (bestB - worldTransformB.GetPosition()) - (normal * volumeB.GetRadius());
		collisionInfo.AddContactPoint(localA, localB, normal, penetration);
		return true;//we�re colliding!
	}
	return false;
}

//AABB - Sphere Collision
bool CollisionDetection::AABBSphereIntersection(const AABBVolume& volumeA, const Transform& worldTransformA,
	const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {
	
	Vector3 boxSize = volumeA.GetHalfDimensions();

	Vector3 delta = worldTransformB.GetPosition() - worldTransformA.GetPosition();

	Vector3 closestPointOnBox = Maths::Clamp(delta, -boxSize, boxSize);

	Vector3 localPoint = delta - closestPointOnBox;
	float distance = localPoint.Length();

	if (distance < volumeB.GetRadius()) {//yes , we�re colliding!
		Vector3 collisionNormal = localPoint.Normalised();
		float penetration = (volumeB.GetRadius() - distance);

		Vector3 localA = Vector3();
		Vector3 localB = -collisionNormal * volumeB.GetRadius();

		collisionInfo.AddContactPoint(localA, localB, collisionNormal, penetration);
		return true;
	}
	return false;
}

//OBB - Sphere Collision
bool  CollisionDetection::OBBSphereIntersection(const OBBVolume& volumeA, const Transform& worldTransformA,
	const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	Vector3 relPos = worldTransformB.GetPosition() - worldTransformA.GetPosition();
	Quaternion tempQuat = worldTransformA.GetOrientation().Conjugate();
	Vector3 spherePos = tempQuat * relPos;

	//Debug::DrawLine(worldTransformA.GetPosition(), spherePos, Vector4(1, 1, 1, 1), 0.1);

	Vector3 boxSize = volumeA.GetHalfDimensions();

	//Vector3 delta = spherePos - worldTransformA.GetPosition();

	Vector3 closestPointOnBox = Maths::Clamp(spherePos, -boxSize, boxSize);

	Vector3 localPoint = spherePos - closestPointOnBox;
	float distance = localPoint.Length();

	if (distance < volumeB.GetRadius()) {//yes , we�re colliding!
		Vector3 collisionNormal = worldTransformA.GetOrientation() * localPoint.Normalised();
		float penetration = (volumeB.GetRadius() - distance);

		Vector3 localA = worldTransformA.GetOrientation() * closestPointOnBox;
		Vector3 localB = -collisionNormal * volumeB.GetRadius();

		collisionInfo.AddContactPoint(localA, localB, collisionNormal, penetration);
		return true;
	}
	return false;
}

//AABB - Capsule Collision
bool CollisionDetection::AABBCapsuleIntersection(
	const CapsuleVolume& volumeA, const Transform& worldTransformA,
	const AABBVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	Vector3 boxSize = volumeB.GetHalfDimensions();

	Vector3 CapsuleOrientation = worldTransformA.GetOrientation() * Vector3(0, volumeA.GetHalfHeight() - volumeA.GetRadius(), 0);

	Vector3 CapsuleTop = worldTransformA.GetPosition() + CapsuleOrientation;

	Vector3 CapsuleBottom = worldTransformA.GetPosition() - CapsuleOrientation;

	Vector3 delta = worldTransformB.GetPosition() - worldTransformA.GetPosition();
	Vector3 closestPointOnBox = Maths::Clamp(-delta, -boxSize, boxSize);

	Vector3 LinePoint = LineClosestPoint(CapsuleTop, CapsuleBottom, worldTransformB.GetPosition() + closestPointOnBox);

	delta = worldTransformB.GetPosition() - LinePoint;
	closestPointOnBox = Maths::Clamp(-delta, -boxSize, boxSize);

	Vector3 localPoint = delta + closestPointOnBox;
	float distance = localPoint.Length();

	if (distance < volumeA.GetRadius()) {//yes , we�re colliding!
		Vector3 collisionNormal = localPoint.Normalised();
		float penetration = (volumeA.GetRadius() - distance);

		Vector3 localA = (LinePoint - worldTransformA.GetPosition()) + (collisionNormal * volumeA.GetRadius());
		Vector3 localB = -collisionNormal * volumeA.GetRadius();

		collisionInfo.AddContactPoint(localA, localB, collisionNormal, penetration);

 		return true;
	}

	return false;
}

Vector3 CollisionDetection::OBBSupport(const Transform& worldTransform, Vector3 worldDir) {
	Vector3 localDir = worldTransform.GetOrientation().Conjugate() * worldDir;
	Vector3 vertex;
	vertex.x = localDir.x < 0 ? -0.5f : 0.5f;
	vertex.y = localDir.y < 0 ? -0.5f : 0.5f;
	vertex.z = localDir.z < 0 ? -0.5f : 0.5f;

	return worldTransform.GetMatrix() * vertex;
}

// OBB on OBB time

bool CollisionDetection::OBBIntersection(const OBBVolume& volumeA, const Transform& worldTransformA,
	const OBBVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	Vector3 bestAxis;
	float penetration = FLT_MAX;

	Vector3 xAxis = Vector3(1, 0, 0);
	Vector3 yAxis = Vector3(0, 1, 0);
	Vector3 zAxis = Vector3(0, 0, 1);
	Quaternion rotA = worldTransformA.GetOrientation();
	Quaternion rotB = worldTransformB.GetOrientation();

	Vector3 AX = rotA * xAxis;
	Vector3 AY = rotA * yAxis;
	Vector3 AZ = rotA * zAxis;
	Vector3 BX = rotB * xAxis;
	Vector3 BY = rotB * yAxis;
	Vector3 BZ = rotB * zAxis;

	Vector3 AXBX = Vector3::Cross(AX, BX);
	Vector3 AYBY = Vector3::Cross(AY, BY);
	Vector3 AZBZ = Vector3::Cross(AZ, BZ);

	Vector3 faces[15] =
	{
		AX, AY, AZ,
		BX, BY, BZ,
		AXBX, Vector3::Cross(AX, BY), Vector3::Cross(AX, BZ),
		Vector3::Cross(AY, BX), AYBY, Vector3::Cross(AY, BZ),
		Vector3::Cross(AZ, BX), Vector3::Cross(AZ, BY), AZBZ
	};

	float distances[15];

	bool isIt = true;

	int iterations = 15;

	if (rotA.x == 0.0f || rotA.y == 0.0f || rotA.z == 0.0f || rotB.x == 0.0f || rotB.y == 0.0f || rotB.z == 0.0f) {
		iterations = 6;
	}

	for (int i = 0; i < iterations && isIt == true; i++) {
		distances[i] = FLT_MAX;

		//Debug::DrawLine(OBBSupport(worldTransformA, faces[i]), OBBSupport(worldTransformA, -faces[i]), Vector4(1, 0, 0, 1), 0.1);
		//Debug::DrawLine(OBBSupport(worldTransformB, faces[i]), OBBSupport(worldTransformB, -faces[i]), Vector4(1, 1, 0, 1), 0.1);

		Vector3 VmaxA = OBBSupport(worldTransformA, faces[i]);
		Vector3 VminA = OBBSupport(worldTransformA, -faces[i]);
		Vector3 VmaxB = OBBSupport(worldTransformB, faces[i]);
		Vector3 VminB = OBBSupport(worldTransformB, -faces[i]);

		float maxA = Vector3::Dot(faces[i], VmaxA);
		float minA = Vector3::Dot(faces[i], VminA);
		float maxB = Vector3::Dot(faces[i], VmaxB);
		float minB = Vector3::Dot(faces[i], VminB);

		if ((minB < minA && minA < maxB) || (minA < minB && minB < maxA)) {
			if ((minB < minA && minA < maxB) && (minB < maxA && maxA < maxB)) {
				distances[i] = maxA - minA;
			}
			else if ((minA < minB && minB < maxA) && (minA < maxB && maxB < maxA)) {
				distances[i] = maxB - minB;
			}
			else if (minB < minA && minA < maxB) {
				distances[i] = maxB - minA;
			}
			else {
				distances[i] = maxA - minB;
			}
		
			if (distances[i] < penetration) {
				penetration = distances[i];
				bestAxis = faces[i];
				if (Vector3::Dot(faces[i], worldTransformA.GetPosition()) > Vector3::Dot(faces[i], worldTransformB.GetPosition())) {
					bestAxis = -bestAxis;
				}
			}
		}
		else {
			//std::cout << "failure on check number " << i << std::endl;
			return false;
		}
	}

	Vector3 localA = OBBSupport(worldTransformA, bestAxis);
	Vector3 localB = OBBSupport(worldTransformB, -bestAxis);

	collisionInfo.AddContactPoint(localA, localB, bestAxis, penetration);
	return true;
}

bool CollisionDetection::OBBAABBIntersection(const OBBVolume& volumeA, const Transform& worldTransformA,
	const AABBVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	Vector3 bestAxis;
	float penetration = FLT_MAX;

	Vector3 xAxis = Vector3(1, 0, 0);
	Vector3 yAxis = Vector3(0, 1, 0);
	Vector3 zAxis = Vector3(0, 0, 1);
	Quaternion rotA = worldTransformA.GetOrientation();
	Quaternion rotB = worldTransformB.GetOrientation();

	Vector3 AX = rotA * xAxis;
	Vector3 AY = rotA * yAxis;
	Vector3 AZ = rotA * zAxis;
	Vector3 BX = rotB * xAxis;
	Vector3 BY = rotB * yAxis;
	Vector3 BZ = rotB * zAxis;

	Vector3 AXBX = Vector3::Cross(AX, BX);
	Vector3 AYBY = Vector3::Cross(AY, BY);
	Vector3 AZBZ = Vector3::Cross(AZ, BZ);

	Vector3 faces[15] =
	{
		AX, AY, AZ,
		BX, BY, BZ,
		AXBX, Vector3::Cross(AX, BY), Vector3::Cross(AX, BZ),
		Vector3::Cross(AY, BX), AYBY, Vector3::Cross(AY, BZ),
		Vector3::Cross(AZ, BX), Vector3::Cross(AZ, BY), AZBZ
	};

	float distances[15];

	bool isIt = true;

	int iterations = 15;

	if (rotA.x == 0.0f || rotA.y == 0.0f || rotA.z == 0.0f || rotB.x == 0.0f || rotB.y == 0.0f || rotB.z == 0.0f) {
		iterations = 6;
	}

	for (int i = 0; i < iterations && isIt == true; i++) {
		distances[i] = FLT_MAX;

		//Debug::DrawLine(OBBSupport(worldTransformA, faces[i]), OBBSupport(worldTransformA, -faces[i]), Vector4(1, 0, 0, 1), 0.1);
		//Debug::DrawLine(OBBSupport(worldTransformB, faces[i]), OBBSupport(worldTransformB, -faces[i]), Vector4(1, 1, 0, 1), 0.1);

		Vector3 VmaxA = OBBSupport(worldTransformA, faces[i]);
		Vector3 VminA = OBBSupport(worldTransformA, -faces[i]);
		Vector3 VmaxB = OBBSupport(worldTransformB, faces[i]);
		Vector3 VminB = OBBSupport(worldTransformB, -faces[i]);

		float maxA = Vector3::Dot(faces[i], VmaxA);
		float minA = Vector3::Dot(faces[i], VminA);
		float maxB = Vector3::Dot(faces[i], VmaxB);
		float minB = Vector3::Dot(faces[i], VminB);

		if ((minB < minA && minA < maxB) || (minA < minB && minB < maxA)) {
			if ((minB < minA && minA < maxB) && (minB < maxA && maxA < maxB)) {
				distances[i] = maxA - minA;
			}
			else if ((minA < minB && minB < maxA) && (minA < maxB && maxB < maxA)) {
				distances[i] = maxB - minB;
			}
			else if (minB < minA && minA < maxB) {
				distances[i] = maxB - minA;
			}
			else {
				distances[i] = maxA - minB;
			}

			if (distances[i] < penetration) {
				penetration = distances[i];
				bestAxis = faces[i];
			}
		}
		else {
			//std::cout << "failure on check number " << i << std::endl;
			return false;
		}
	}

	Vector3 localA = OBBSupport(worldTransformA, bestAxis);
	Vector3 localB = Vector3();

	collisionInfo.AddContactPoint(localA, localB, bestAxis, penetration);
	return true;
}

bool CollisionDetection::OBBCapsuleIntersection(
	const CapsuleVolume& volumeA, const Transform& worldTransformA,
	const OBBVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	return false;
}


Vector3 CollisionDetection::LineClosestPoint(Vector3 A, Vector3 B, Vector3 Point) {
	Vector3 line = B - A;
	float proportion = Maths::Clamp((Vector3::Dot(Point - A, line) / Vector3::Dot(line, line)), 0.0f, 1.0f);
	return A + Vector3(line.x*proportion, line.y * proportion, line.z * proportion);
}

//Sphere - Capsule Collision
bool CollisionDetection::SphereCapsuleIntersection(
	const CapsuleVolume& volumeA, const Transform& worldTransformA,
	const SphereVolume& volumeB, const Transform& worldTransformB, CollisionInfo& collisionInfo) {

	float radii = volumeA.GetRadius() + volumeB.GetRadius();

	Vector3 CapsuleOrientation = worldTransformA.GetOrientation() * Vector3(0, volumeA.GetHalfHeight() - volumeA.GetRadius(), 0);

	Vector3 CapsuleTop = worldTransformA.GetPosition() + CapsuleOrientation;

	Vector3 CapsuleBottom = worldTransformA.GetPosition() - CapsuleOrientation;

	Vector3 LinePoint = LineClosestPoint(CapsuleTop, CapsuleBottom, worldTransformB.GetPosition());

	Vector3 delta = worldTransformB.GetPosition() - LinePoint;
	float deltaLength = delta.Length();

	if (deltaLength < radii) {
		float penetration = (radii - deltaLength);
		Vector3 normal = delta.Normalised();
		Vector3 localA = (LinePoint - worldTransformA.GetPosition()) + (normal * volumeA.GetRadius());
		Vector3 localB = -normal * volumeB.GetRadius();
		collisionInfo.AddContactPoint(localA, localB, normal, penetration);
		//Debug::DrawLine(worldTransformB.GetPosition(), LinePoint, Vector4(1, 0, 0, 1), 5);
		return true;//we�re colliding!
	}
	return false;
}

Matrix4 GenerateInverseView(const Camera &c) {
	float pitch = c.GetPitch();
	float yaw	= c.GetYaw();
	Vector3 position = c.GetPosition();

	Matrix4 iview =
		Matrix4::Translation(position) *
		Matrix4::Rotation(-yaw, Vector3(0, -1, 0)) *
		Matrix4::Rotation(-pitch, Vector3(-1, 0, 0));

	return iview;
}

Matrix4 GenerateInverseProjection(float aspect, float nearPlane, float farPlane, float fov) {
	float negDepth = nearPlane - farPlane;

	float invNegDepth = negDepth / (2 * (farPlane * nearPlane));

	Matrix4 m;

	float h = 1.0f / tan(fov*PI_OVER_360);

	m.array[0][0] = aspect / h;
	m.array[1][1] = tan(fov * PI_OVER_360);
	m.array[2][2] = 0.0f;

	m.array[2][3] = invNegDepth;//// +PI_OVER_360;
	m.array[3][2] = -1.0f;
	m.array[3][3] = (0.5f / nearPlane) + (0.5f / farPlane);

	return m;
}

Vector3 CollisionDetection::Unproject(const Vector3& screenPos, const Camera& cam) {
	Vector2 screenSize = Window::GetWindow()->GetScreenSize();

	float aspect	= screenSize.x / screenSize.y;
	float fov		= cam.GetFieldOfVision();
	float nearPlane = cam.GetNearPlane();
	float farPlane  = cam.GetFarPlane();

	//Create our inverted matrix! Note how that to get a correct inverse matrix,
	//the order of matrices used to form it are inverted, too.
	Matrix4 invVP = GenerateInverseView(cam) * GenerateInverseProjection(aspect, fov, nearPlane, farPlane);

	Matrix4 proj  = cam.BuildProjectionMatrix(aspect);

	//Our mouse position x and y values are in 0 to screen dimensions range,
	//so we need to turn them into the -1 to 1 axis range of clip space.
	//We can do that by dividing the mouse values by the width and height of the
	//screen (giving us a range of 0.0 to 1.0), multiplying by 2 (0.0 to 2.0)
	//and then subtracting 1 (-1.0 to 1.0).
	Vector4 clipSpace = Vector4(
		(screenPos.x / (float)screenSize.x) * 2.0f - 1.0f,
		(screenPos.y / (float)screenSize.y) * 2.0f - 1.0f,
		(screenPos.z),
		1.0f
	);

	//Then, we multiply our clipspace coordinate by our inverted matrix
	Vector4 transformed = invVP * clipSpace;

	//our transformed w coordinate is now the 'inverse' perspective divide, so
	//we can reconstruct the final world space by dividing x,y,and z by w.
	return Vector3(transformed.x / transformed.w, transformed.y / transformed.w, transformed.z / transformed.w);
}

Ray CollisionDetection::BuildRayFromMouse(const Camera& cam) {
	Vector2 screenMouse = Window::GetMouse()->GetAbsolutePosition();
	Vector2 screenSize	= Window::GetWindow()->GetScreenSize();

	//We remove the y axis mouse position from height as OpenGL is 'upside down',
	//and thinks the bottom left is the origin, instead of the top left!
	Vector3 nearPos = Vector3(screenMouse.x,
		screenSize.y - screenMouse.y,
		-0.99999f
	);

	//We also don't use exactly 1.0 (the normalised 'end' of the far plane) as this
	//causes the unproject function to go a bit weird. 
	Vector3 farPos = Vector3(screenMouse.x,
		screenSize.y - screenMouse.y,
		0.99999f
	);

	Vector3 a = Unproject(nearPos, cam);
	Vector3 b = Unproject(farPos, cam);
	Vector3 c = b - a;

	c.Normalise();

	return Ray(cam.GetPosition(), c);
}

//http://bookofhook.com/mousepick.pdf
Matrix4 CollisionDetection::GenerateInverseProjection(float aspect, float fov, float nearPlane, float farPlane) {
	Matrix4 m;

	float t = tan(fov*PI_OVER_360);

	float neg_depth = nearPlane - farPlane;

	const float h = 1.0f / t;

	float c = (farPlane + nearPlane) / neg_depth;
	float e = -1.0f;
	float d = 2.0f*(nearPlane*farPlane) / neg_depth;

	m.array[0][0] = aspect / h;
	m.array[1][1] = tan(fov * PI_OVER_360);
	m.array[2][2] = 0.0f;

	m.array[2][3] = 1.0f / d;

	m.array[3][2] = 1.0f / e;
	m.array[3][3] = -c / (d * e);

	return m;
}

/*
And here's how we generate an inverse view matrix. It's pretty much
an exact inversion of the BuildViewMatrix function of the Camera class!
*/
Matrix4 CollisionDetection::GenerateInverseView(const Camera &c) {
	float pitch = c.GetPitch();
	float yaw	= c.GetYaw();
	Vector3 position = c.GetPosition();

	Matrix4 iview =
		Matrix4::Translation(position) *
		Matrix4::Rotation(yaw, Vector3(0, 1, 0)) *
		Matrix4::Rotation(pitch, Vector3(1, 0, 0));

	return iview;
}


/*
If you've read through the Deferred Rendering tutorial you should have a pretty
good idea what this function does. It takes a 2D position, such as the mouse
position, and 'unprojects' it, to generate a 3D world space position for it.

Just as we turn a world space position into a clip space position by multiplying
it by the model, view, and projection matrices, we can turn a clip space
position back to a 3D position by multiply it by the INVERSE of the
view projection matrix (the model matrix has already been assumed to have
'transformed' the 2D point). As has been mentioned a few times, inverting a
matrix is not a nice operation, either to understand or code. But! We can cheat
the inversion process again, just like we do when we create a view matrix using
the camera.

So, to form the inverted matrix, we need the aspect and fov used to create the
projection matrix of our scene, and the camera used to form the view matrix.

*/
Vector3	CollisionDetection::UnprojectScreenPosition(Vector3 position, float aspect, float fov, const Camera &c) {
	//Create our inverted matrix! Note how that to get a correct inverse matrix,
	//the order of matrices used to form it are inverted, too.
	Matrix4 invVP = GenerateInverseView(c) * GenerateInverseProjection(aspect, fov, c.GetNearPlane(), c.GetFarPlane());

	Vector2 screenSize = Window::GetWindow()->GetScreenSize();

	//Our mouse position x and y values are in 0 to screen dimensions range,
	//so we need to turn them into the -1 to 1 axis range of clip space.
	//We can do that by dividing the mouse values by the width and height of the
	//screen (giving us a range of 0.0 to 1.0), multiplying by 2 (0.0 to 2.0)
	//and then subtracting 1 (-1.0 to 1.0).
	Vector4 clipSpace = Vector4(
		(position.x / (float)screenSize.x) * 2.0f - 1.0f,
		(position.y / (float)screenSize.y) * 2.0f - 1.0f,
		(position.z) - 1.0f,
		1.0f
	);

	//Then, we multiply our clipspace coordinate by our inverted matrix
	Vector4 transformed = invVP * clipSpace;

	//our transformed w coordinate is now the 'inverse' perspective divide, so
	//we can reconstruct the final world space by dividing x,y,and z by w.
	return Vector3(transformed.x / transformed.w, transformed.y / transformed.w, transformed.z / transformed.w);
}
