#pragma once
#include "Transform.h"
#include "CollisionVolume.h"
#include "PhysicsObject.h"

#include "AABBVolume.h"
#include "OBBVolume.h"
#include "SphereVolume.h"
#include "CapsuleVolume.h"

using std::vector;

namespace NCL::CSC8503 {
	class NetworkObject;
	class RenderObject;
	class PhysicsObject;

	class GameObject	{
	public:
		GameObject(std::string name = "");
		~GameObject();

		void SetBoundingVolume(CollisionVolume* vol) {
			boundingVolume = vol;
		}

		const CollisionVolume* GetBoundingVolume() const {
			return boundingVolume;
		}

		bool IsActive() const {
			return isActive;
		}

		void deactivate() {
			isActive = false;
		}

		Transform& GetTransform() {
			return transform;
		}

		RenderObject* GetRenderObject() const {
			return renderObject;
		}

		PhysicsObject* GetPhysicsObject() const {
			return physicsObject;
		}

		NetworkObject* GetNetworkObject() const {
			return networkObject;
		}

		void SetRenderObject(RenderObject* newObject) {
			renderObject = newObject;
		}

		void SetPhysicsObject(PhysicsObject* newObject) {
			physicsObject = newObject;
		}

		const std::string& GetName() const {
			return name;
		}

		int getScore() {
			return score;
		}

		int getItemCount() {
			return itemCount;
		}

		void changeItemCount(int i) {
			itemCount += i;
		}
		
		void changeScore(int i) {
			score += i;
		}

		void setScore(int i) {
			score = i;
		}

		virtual void OnCollisionBegin(GameObject* otherObject) {
			//std::cout << "OnCollisionBegin event occured!\n";

			if (GetRayLevel() == 6 && otherObject->GetRayLevel() == 10 ) { // Score 10
				otherObject->deactivate();
				otherObject->SetBoundingVolume(nullptr);
				changeScore(10);
				changeItemCount(1);
			}
			else if (GetRayLevel() == 6 && otherObject->GetRayLevel() == 30) { // Score 30
				otherObject->deactivate();
				otherObject->SetBoundingVolume(nullptr);
				changeScore(30);
				changeItemCount(1);
			}
			else if (GetRayLevel() == 6 && otherObject->GetRayLevel() == 50) { // Score 50
				otherObject->deactivate();
				otherObject->SetBoundingVolume(nullptr);
				changeScore(50);
				changeItemCount(1);
			}
			else if (GetRayLevel() == 6 && otherObject->GetRayLevel() == 45) { // Goat - Alarm
				setHadCollision(true);
				setCollisionArea(GetTransform().GetPosition());
			}
			else if (GetRayLevel() == 6 && otherObject->GetRayLevel() == 666) { // Goat - GOOSE
				changeScore(-50);
				GetTransform().SetPosition(Vector3(30, 8, 30));
				otherObject->GetTransform().SetPosition(Vector3(360, 6, 360));
				otherObject->GetPhysicsObject()->SetLinearVelocity(Vector3(0, 0, 0));
				otherObject->setGooseGotMe(true);
			}
			else if (GetRayLevel() == 40) {
				Vector3 speed1 = physicsObject->GetLinearVelocity();
				Vector3 speed2 = otherObject->GetPhysicsObject()->GetLinearVelocity();
				if (speed1.x - speed2.x >= 15 || speed1.y - speed2.y >= 15 || speed1.z - speed2.z >= 15) {
					deactivate();
					SetBoundingVolume(nullptr);
				}
			}
			else if (GetRayLevel() == 8 && otherObject->GetRayLevel() == 9) {
				deactivate();
				SetBoundingVolume(nullptr);
			}
		}

		virtual void OnCollisionEnd(GameObject* otherObject) {
			//std::cout << "OnCollisionEnd event occured!\n";
		}

		bool GetBroadphaseAABB(Vector3& outsize) const;

		void UpdateBroadphaseAABB();

		void SetWorldID(int newID) {
			worldID = newID;
		}

		int		GetWorldID() const {
			return worldID;
		}

		void SetRayLevel(int in) {
			rayLevel = in;
		}

		int GetRayLevel() {
			return rayLevel;
		}

		float getRestitution() {
			return restitution;
		}
		void setRestitution(float i) {
			restitution = i;
		}

		Vector4 getDefaultColour() {
			return defaultColour;
		}
		void setDefaultColour(Vector4 input) {
			defaultColour = input;
		}

		void setHadCollision(bool entry) {
			hadCollision = entry;
		}
		bool getHadCollision() {
			return hadCollision;
		}

		void setCollisionArea(Vector3 input) {
			collisionArea = input;
		}
		Vector3 getcollisionArea() {
			return collisionArea;
		}

		void setGooseGotMe(bool input) {
			gooseGotMe = input;
		}
		bool getGooseGotMe() {
			return gooseGotMe;
		}

	protected:
		Transform			transform;

		CollisionVolume*	boundingVolume;
		PhysicsObject*		physicsObject;
		RenderObject*		renderObject;
		NetworkObject*		networkObject;

		bool		isActive;
		int			worldID;
		std::string	name;

		Vector3 broadphaseAABB;

		float		restitution;

		int			rayLevel;

		int			score;
		int			itemCount;

		Vector4		defaultColour;

		bool		hadCollision;
		Vector3		collisionArea;

		bool		gooseGotMe = false;
	};
}

