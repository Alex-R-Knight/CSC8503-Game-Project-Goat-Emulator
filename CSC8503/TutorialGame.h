#pragma once
#include "GameTechRenderer.h"
#ifdef USEVULKAN
#include "GameTechVulkanRenderer.h"
#endif
#include "PhysicsSystem.h"

#include "StateGameObject.h"

#include "BehaviourNode.h"
#include "BehaviourSelector.h"
#include "BehaviourSequence.h"
#include "BehaviourAction.h"

namespace NCL {
	namespace CSC8503 {
		class PositionConstraint;
		class TutorialGame		{
		public:
			TutorialGame();
			~TutorialGame();

			virtual void UpdateGame(float dt);

			void InitWorld();

			int getEndScore() { return lockedObject->getScore(); };
			int getEndItems() { return itemCounter- lockedObject->getItemCount(); };
			float getTime() { return gameTime; };

		protected:
			void InitialiseAssets();

			void InitCamera();
			void UpdateKeys(float dt);

			float gameTime;
			float itemCounter;

			/*
			These are some of the world/object creation functions I created when testing the functionality
			in the module. Feel free to mess around with them to see different objects being created in different
			test scenarios (constraints, collision types, and so on). 
			*/
			void InitGameExamples();

			void InitGameWorld(const std::string& filename);
			void InitGameWorld2(const std::string& filename);

			void InitSphereGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, float radius);
			void InitMixedGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing);
			void InitCubeGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, const Vector3& cubeDims);

			void BridgeConstraintTest();

			void InitDefaultFloor();

			bool SelectObject();
			void MoveSelectedObject();
			void DebugObjectMovement();
			void LockedObjectMovement(float dt);

			void scoreWork();
			int gameScore;

			StateGameObject* AddStateObjectToWorld(const Vector3& position);
			StateGameObject* testStateObject;

			GameObject* AddFloorToWorld(const Vector3& position);
			GameObject* AddCustomFloorToWorld(const Vector3& position, const Vector3& floorSize);
			GameObject* AddSphereToWorld(const Vector3& position, float radius, float inverseMass = 10.0f);
			GameObject* AddCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass = 5.0f);
			GameObject* AddOBBToWorld(const Vector3& position, Vector3 dimensions, float inverseMass = 1.0f);

			GameObject* AddKeyToWorld(const Vector3& position, Vector3 dimensions = Vector3(1, 1, 0.5), float inverseMass = 6.0f);

			GameObject* AddTargetCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass = 0.1f);
			GameObject* AddAlarmCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass = 3.0f);

			GameObject* AddCrateToWorld(const Vector3& position, Vector3 dimensions = Vector3(2, 2, 2), float inverseMass = 0.3f);

			GameObject* AddWallToWorld(const Vector3& position, Vector3 dimensions);
			GameObject* AddWall2ToWorld(const Vector3& position, Vector3 dimensions);
			GameObject* AddDoorToWorld(const Vector3& position, Vector3 dimensions);

			void PlaceAlarm(Vector3 pos);

			void PlaceLoot1(Vector3 pos);
			void PlaceLoot2(Vector3 pos);
			void PlaceLoot3(Vector3 pos);

			void PlaceBarricade(Vector3 pos);
			void PlaceBarricade2(Vector3 pos);

			void PlaceCrates(Vector3 pos);
			void PlaceCrates2(Vector3 pos);

			PositionConstraint* goatConstraint;

			GameObject* AddTarget10ToWorld(const Vector3& position, float radius, float inverseMass = 10.0f);
			GameObject* AddTarget30ToWorld(const Vector3& position, float radius, float inverseMass = 10.0f);
			GameObject* AddTarget50ToWorld(const Vector3& position, float radius, float inverseMass = 10.0f);

			GameObject* AddPlayerToWorld(const Vector3& position);
			GameObject* AddGooseToWorld(const Vector3& position);
			GameObject* AddEnemyToWorld(const Vector3& position);
			GameObject* AddBonusToWorld(const Vector3& position);

			Matrix3 GoatRotationMatrix(const Vector3& direction);

#ifdef USEVULKAN
			GameTechVulkanRenderer*	renderer;
#else
			GameTechRenderer* renderer;
#endif
			PhysicsSystem*		physics;
			GameWorld*			world;

			bool useGravity;
			bool inSelectionMode;

			float		forceMagnitude;

			GameObject* selectionObject = nullptr;

			MeshGeometry*	capsuleMesh = nullptr;
			MeshGeometry*	cubeMesh	= nullptr;
			MeshGeometry*	sphereMesh	= nullptr;

			TextureBase*	basicTex	= nullptr;
			ShaderBase*		basicShader = nullptr;

			//Coursework Meshes
			MeshGeometry*	charMesh	= nullptr;
			MeshGeometry*	gooseMesh	= nullptr;
			MeshGeometry*	enemyMesh	= nullptr;
			MeshGeometry*	bonusMesh	= nullptr;

			//Coursework Additional functionality	
			GameObject* lockedObject	= nullptr;
			Vector3 lockedOffset		= Vector3(0, 14, 20);
			float orbitScalar		= 30.0f;
			float orbitScalarMax = 30.0f;
			float orbitScalarMin = 2.0f;

			float thirdPersonYScalar = 1;
			float thirdPersonXScalar = 1.25;
			float thirdPersonZScalar = 4;

			void LockCameraToObject(GameObject* o) {
				lockedObject = o;
			}

			GameObject* GOOSE = nullptr;

			GameObject* grabObject = nullptr;

			GameObject* objClosest = nullptr;


			//Behaviour Tree
			void GooseBehaviourTree();
			bool GooseGoatCheck();
			void gooseMoveToNode(Vector3 destination);
			bool gooseNavTarget;
			Vector3 TestPathfinding(Vector3 destination);

			void updateTree(float dt);

			Vector3 investigateTarget;
			Vector3 moveTarget;
			BehaviourSequence* rootSequence;
			float cooldownTimer;
			Vector3 PatrolPath[5] = {
				Vector3(180, 3, 320),
				Vector3(220, 3, 360),
				Vector3(320, 3, 360),
				Vector3(260, 3, 300),
				Vector3(320, 3, 260)
			};
			Vector3 patrolTarget;


			// Third Person Camera Tests
			bool thirdPerson = false;

			Vector3 orbitCameraProcess(Vector3 objPos);
			Vector3 thirdPersonCameraProcess(Vector3 objPos);
			void cameraInterpolation(Vector3 target, float dt);
			float cameraInterpBaseSpeed = 0.5f;

			Quaternion thirdPersonRotationCalc(GameWorld* world, GameObject* object, Camera* cam, Vector3 camPos);
		};
	}
}

