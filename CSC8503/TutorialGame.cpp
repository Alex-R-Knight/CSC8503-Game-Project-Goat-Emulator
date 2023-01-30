#include "TutorialGame.h"
#include "GameWorld.h"
#include "PhysicsObject.h"
#include "RenderObject.h"
#include "TextureLoader.h"

#include "NavigationGrid.h"
#include "NavigationMesh.h"

#include "Window.h"

#include <fstream>
#include "Assets.h"

#include "PositionConstraint.h"
#include "OrientationConstraint.h"
#include "StateGameObject.h"

#include "BehaviourNode.h"
#include "BehaviourSelector.h"
#include "BehaviourSequence.h"
#include "BehaviourAction.h"

#include "Maths.h"


using namespace NCL;
using namespace CSC8503;

TutorialGame::TutorialGame()	{
	world		= new GameWorld();
#ifdef USEVULKAN
	renderer	= new GameTechVulkanRenderer(*world);
#else 
	renderer = new GameTechRenderer(*world);
#endif

	physics		= new PhysicsSystem(*world);

	forceMagnitude	= 10.0f;
	useGravity		= true;
	physics->UseGravity(useGravity);
	inSelectionMode = false;
	gooseNavTarget = false;

	InitialiseAssets();
}

/*

Each of the little demo scenarios used in the game uses the same 2 meshes, 
and the same texture and shader. There's no need to ever load in anything else
for this module, even in the coursework, but you can add it if you like!

*/
void TutorialGame::InitialiseAssets() {
	cubeMesh	= renderer->LoadMesh("cube.msh");
	sphereMesh	= renderer->LoadMesh("sphere.msh");
	charMesh	= renderer->LoadMesh("goat.msh");
	gooseMesh	= renderer->LoadMesh("goose.msh");
	enemyMesh	= renderer->LoadMesh("Keeper.msh");
	bonusMesh	= renderer->LoadMesh("apple.msh");
	capsuleMesh = renderer->LoadMesh("capsule.msh");

	basicTex	= renderer->LoadTexture("checkerboard.png");
	basicShader = renderer->LoadShader("scene.vert", "scene.frag");
	
	InitCamera();
	InitWorld();
}

TutorialGame::~TutorialGame()	{
	delete cubeMesh;
	delete sphereMesh;
	delete charMesh;
	delete gooseMesh;
	delete enemyMesh;
	delete bonusMesh;

	delete basicTex;
	delete basicShader;

	delete physics;
	delete renderer;
	delete world;
}

void TutorialGame::updateTree(float dt) {
	BehaviourState state = Ongoing;
	state = rootSequence->Execute(dt);
	if (state == Success || state == Failure) {
		rootSequence->Reset();
	}
}

void TutorialGame::UpdateGame(float dt) {
	if (lockedObject != nullptr) {

		world->GetMainCamera()->ThirdPersonUpdateRot();

		Vector3 objPos = lockedObject->GetTransform().GetPosition();

		Vector3 camPos;
		
		if (!thirdPerson) {
			camPos = orbitCameraProcess(objPos);
			cameraInterpolation(camPos, dt);
		}
		else {
			camPos = thirdPersonCameraProcess(objPos);
			cameraInterpolation(camPos, dt);
			camPos = world->GetMainCamera()->GetPosition();
			//lockedObject->GetTransform().SetOrientation(thirdPersonRotationCalc(world, lockedObject, world->GetMainCamera(), camPos));

			Quaternion goatTargetRotation = thirdPersonRotationCalc(world, lockedObject, world->GetMainCamera(), camPos);
			Quaternion goatStartRotation = lockedObject->GetTransform().GetOrientation();
			Quaternion goatRealRotation = Quaternion::Lerp(goatStartRotation, goatTargetRotation, 0.25f);
			lockedObject->GetTransform().SetOrientation(goatRealRotation);
		}
	}

	gameTime += dt;

	updateTree(dt);
	//Debug::Print("Cooldown timer: " + std::to_string(cooldownTimer), Vector2(2, 15));

	if (testStateObject) {
		testStateObject->Update(dt);
	}

	UpdateKeys(dt);

	//if (useGravity) {
	//	Debug::Print("(G)ravity on", Vector2(5, 95), Debug::RED);
	//}
	//else {
	//	Debug::Print("(G)ravity off", Vector2(5, 95), Debug::RED);
	//}

	RayCollision closestCollision;
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::K) && selectionObject) {
		Vector3 rayPos;
		Vector3 rayDir;

		rayDir = selectionObject->GetTransform().GetOrientation() * Vector3(0, 0, -1);

		rayPos = selectionObject->GetTransform().GetPosition();

		Ray r = Ray(rayPos, rayDir);

		if (world->Raycast(r, closestCollision, true, selectionObject)) {
			if (objClosest) {
				objClosest->GetRenderObject()->SetColour(Vector4(1, 1, 1, 1));
			}
			objClosest = (GameObject*)closestCollision.node;

			objClosest->GetRenderObject()->SetColour(Vector4(1, 0, 1, 1));

			Debug::DrawLine(Vector3(selectionObject->GetTransform().GetPosition()), Vector3(objClosest->GetTransform().GetPosition()), Vector4(1, 0, 0, 1), 5);
		}
		else {
			Debug::DrawLine(Vector3(selectionObject->GetTransform().GetPosition()), rayPos + rayDir*100, Vector4(1, 0, 0, 1), 5);
		}
	}
	// RayLevel based raycasting
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::J) && selectionObject) {
		Vector3 rayPos;
		Vector3 rayDir;

		rayDir = selectionObject->GetTransform().GetOrientation() * Vector3(0, 0, -1);

		rayPos = selectionObject->GetTransform().GetPosition();

		Ray r = Ray(rayPos, rayDir);

		if (world->RaycastLevel(r, selectionObject->GetRayLevel(), closestCollision, true, selectionObject)) {
			if (objClosest) {
				objClosest->GetRenderObject()->SetColour(Vector4(1, 1, 1, 1));
			}
			objClosest = (GameObject*)closestCollision.node;

			objClosest->GetRenderObject()->SetColour(Vector4(1, 0, 1, 1));

			Debug::DrawLine(Vector3(selectionObject->GetTransform().GetPosition()), Vector3(objClosest->GetTransform().GetPosition()), Vector4(1, 0, 0, 1), 5);
		}
		else {
			Debug::DrawLine(Vector3(selectionObject->GetTransform().GetPosition()), rayPos + rayDir * 100, Vector4(1, 0, 0, 1), 5);
		}
	}

	//Debug::DrawLine(Vector3(), Vector3(0, 100, 0), Vector4(1, 0, 0, 1));

	SelectObject();
	MoveSelectedObject();

	world->UpdateWorld(dt);
	renderer->Update(dt);
	physics->Update(dt);

	scoreWork();

	renderer->Render();
	Debug::UpdateRenderables(dt);
}

Quaternion TutorialGame::thirdPersonRotationCalc(GameWorld* world, GameObject* object, Camera* cam, Vector3 camPos) {
	Vector2 screenSize = Window::GetWindow()->GetScreenSize();
	Vector3 nearPos = Vector3(screenSize.x / 2,
		screenSize.y / 2,
		-0.99999f
	);
	Vector3 farPos = Vector3(screenSize.x / 2,
		screenSize.y / 2,
		0.99999f
	);

	Vector3 a = CollisionDetection::Unproject(nearPos, *cam);
	Vector3 b = CollisionDetection::Unproject(farPos, *cam);
	Vector3 c = b - a;
	c.Normalise();

	Ray ray = Ray(camPos, c);
	RayCollision aimCollision;
	Quaternion goatStartRotation;
	if (world->Raycast(ray, aimCollision, true, object)) {
		Vector3 collisionVector = aimCollision.collidedAt - object->GetTransform().GetPosition();

		float theta = atan2(collisionVector.z, collisionVector.x) * (180 / PI);
		Quaternion goatTargetRotation = Quaternion(Matrix4::Rotation(-theta - 90, Vector3(0, 1, 0)));
		goatStartRotation = object->GetTransform().GetOrientation();
		Quaternion goatRealRotation = Quaternion::Lerp(goatStartRotation, goatTargetRotation, 0.5f);
		return goatRealRotation;
	}
	Matrix4 Yaw = cam->GetRotationYaw();
	return Quaternion(Yaw);
}

void TutorialGame::cameraInterpolation(Vector3 target, float dt) {
	Camera* currentCamera = world->GetMainCamera();
	Vector3 currentCamPos = currentCamera->GetPosition();
	Vector3 movement = target - currentCamPos;
	float movementLength = movement.Length();

	Vector3 output = currentCamPos + movement * cameraInterpBaseSpeed;
	currentCamera->SetPosition(output);
}

Vector3 TutorialGame::orbitCameraProcess(Vector3 objPos) {
	Matrix4 Pitch = world->GetMainCamera()->GetRotationPitch();
	Matrix4 Yaw = world->GetMainCamera()->GetRotationYaw();
	Quaternion rotationAmount = Quaternion(Yaw) * Quaternion(Pitch);

	orbitScalar -= Window::GetMouse()->GetWheelMovement();
	orbitScalar = Maths::Clamp(orbitScalar, orbitScalarMin, orbitScalarMax);

	Vector3 camPos = objPos + rotationAmount * Vector3(0, 0, orbitScalar);
	RayCollision cameraCollision;
	Ray r = Ray(objPos + rotationAmount * Vector3(0, 0, 1.5), rotationAmount * Vector3(0, 0, 1));
	if (world->Raycast(r, cameraCollision, true, lockedObject)) {
		float distance = cameraCollision.rayDistance;
		if (distance < orbitScalar) {
			camPos = objPos + rotationAmount * Vector3(0, 0, distance);
		}
	}
	return camPos;
}

Vector3 TutorialGame::thirdPersonCameraProcess(Vector3 objPos) {
	Matrix4 Pitch = world->GetMainCamera()->GetRotationPitch();
	Matrix4 Yaw = world->GetMainCamera()->GetRotationYaw();
	Vector3 dispVector = Vector3(thirdPersonXScalar, 0, thirdPersonZScalar);
	float dispLength = dispVector.Length();
	objPos = objPos + Vector3(0, thirdPersonYScalar, 0);
	Quaternion rotationAmount = Quaternion(Yaw) * Quaternion(Pitch);

	Vector3 camPos = objPos + rotationAmount * dispVector;

	RayCollision cameraCollision;
	Ray r = Ray(objPos, rotationAmount * dispVector);
	if (world->Raycast(r, cameraCollision, true, lockedObject)) {
		float distance = cameraCollision.rayDistance;
		if (distance < dispLength) {
			//std::cout << dispLength << ", " << distance << "\n";
			camPos = objPos + rotationAmount * (dispVector.Normalised() * distance);
		}
	}
	return camPos;
}

void TutorialGame::scoreWork() {

	Debug::Print("Current Score: " + std::to_string(lockedObject->getScore()), Vector2(2, 5));
	Debug::Print("Remaining Items: " + std::to_string(itemCounter - lockedObject->getItemCount()), Vector2(2, 9));
	Debug::Print("Remaining Time: " + std::to_string(200 - gameTime), Vector2(2, 13));
}

void TutorialGame::UpdateKeys(float dt) {
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F1)) {
		InitWorld(); //We can reset the simulation at any time with F1
		selectionObject = nullptr;
	}

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F2)) {
		InitCamera(); //F2 will reset the camera to a specific default place
	}

	//if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::G)) {
	//	useGravity = !useGravity; //Toggle gravity!
	//	physics->UseGravity(useGravity);
	//}
	//Running certain physics updates in a consistent order might cause some
	//bias in the calculations - the same objects might keep 'winning' the constraint
	//allowing the other one to stretch too much etc. Shuffling the order so that it
	//is random every frame can help reduce such bias.
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F9)) {
		world->ShuffleConstraints(true);
	}
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F10)) {
		world->ShuffleConstraints(false);
	}

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F7)) {
		world->ShuffleObjects(true);
	}
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::F8)) {
		world->ShuffleObjects(false);
	}

	if (lockedObject) {
		LockedObjectMovement(dt);
	}
	else {
		DebugObjectMovement();
	}
}

void TutorialGame::LockedObjectMovement(float dt) {
	Vector3 pos = lockedObject->GetTransform().GetPosition();
	//Debug::Print("You're at:" + std::to_string(pos.x) + "," + std::to_string(pos.y) + "," + std::to_string(pos.z), Vector2(3, 20));



	Matrix4 view		= world->GetMainCamera()->BuildViewMatrix();
	Matrix4 camWorld	= view.Inverse();

	Quaternion ObjectOrientation = lockedObject->GetTransform().GetOrientation();

	Vector3 CamOffset = Vector3(-15, 0, 0);


	Quaternion Yaw = Quaternion(world->GetMainCamera()->GetRotationYaw());
	Vector3 testEuler = Yaw.ToEuler();

	bool onFloor = false;

	RayCollision floorCollision;
	Ray r = Ray(lockedObject->GetTransform().GetPosition() + Vector3(0, -1, 0), Vector3(0, -1, 0));

	if (world->Raycast(r, floorCollision, true, selectionObject)) {
		float distance = floorCollision.rayDistance;
		if (distance < 1) {
			onFloor = true;
			//Debug::Print("You are on the floor!", Vector2(3, 20));
		}
	}

	Vector3 startVelocity = lockedObject->GetPhysicsObject()->GetLinearVelocity();
	Vector3 endVelocity = Vector3(0, 0, 0);

	bool directionInput = false;
	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::W)) {
		Vector3 trajectory = onFloor ? Vector3(0, 0, -15) : Vector3(0, 0, -7);
		lockedObject->GetPhysicsObject()->AddForce(Yaw * trajectory);
		endVelocity = endVelocity + Yaw * Vector3(0, 0, -1);
		directionInput = true;
	}

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::S)) {
		Vector3 trajectory = onFloor ? Vector3(0, 0, 15) : Vector3(0, 0, 7);
		lockedObject->GetPhysicsObject()->AddForce(Yaw * trajectory);
		endVelocity = endVelocity + Yaw * Vector3(0, 0, 1);
		directionInput = true;
	}

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::A)) {
		Vector3 trajectory = onFloor ? Vector3(-15, 0, 0) : Vector3(-7, 0, 0);
		lockedObject->GetPhysicsObject()->AddForce(Yaw * trajectory);
		endVelocity = endVelocity + Yaw * Vector3(-1, 0, 0);
		directionInput = true;
	}

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::D)) {
		Vector3 trajectory = onFloor ? Vector3(15, 0, 0) : Vector3(15, 0, 0);
		lockedObject->GetPhysicsObject()->AddForce(Yaw * trajectory);
		endVelocity = endVelocity + Yaw * Vector3(1, 0, 0);
		directionInput = true;
	}
	if (!directionInput && onFloor) {
		float scalar = (1 - dt);
		lockedObject->GetPhysicsObject()->SetLinearVelocity(Vector3(startVelocity.x*scalar, startVelocity.y, startVelocity.z*scalar));
	}

	if (directionInput && (endVelocity.Normalised() - startVelocity.Normalised()).Length() > 1.25) {
		//lockedObject->GetPhysicsObject()->SetLinearVelocity(startVelocity* (1 - dt));
		lockedObject->GetPhysicsObject()->AddForce(endVelocity.Normalised()*10);
		//std::cout << "its happening\n";
	}

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::SPACE) && onFloor == true) {
		lockedObject->GetPhysicsObject()->ApplyLinearImpulse(ObjectOrientation * Vector3(0, 25, 0));
	}

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::SHIFT) && onFloor == true) {
		lockedObject->GetPhysicsObject()->ApplyLinearImpulse(ObjectOrientation * Vector3(0, 10, -30));
	}

	//Camera toggle
	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::R)) {
		thirdPerson = !thirdPerson;
	}

	bool manualTurning = false;
	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::Q) && !thirdPerson) {
		manualTurning = true;
		lockedObject->GetPhysicsObject()->AddTorque(Vector3(0.0f, 5.0f, 0.0f));
	}

	if (Window::GetKeyboard()->KeyDown(KeyboardKeys::E) && !thirdPerson) {
		manualTurning = true;
		lockedObject->GetPhysicsObject()->AddTorque(Vector3(0.0f, -5.0f, 0.0f));
	}

	if (!manualTurning && !thirdPerson) {
		Vector3 currentVelocity = lockedObject->GetPhysicsObject()->GetLinearVelocity();
		float theta = atan2(currentVelocity.z, currentVelocity.x)*(180/PI);
		Quaternion goatTargetRotation = Quaternion(Matrix4::Rotation(-theta-90, Vector3(0, 1, 0)));
		Quaternion goatStartRotation = lockedObject->GetTransform().GetOrientation();
		Quaternion goatRealRotation = Quaternion::Lerp(goatStartRotation, goatTargetRotation, 0.5f);
		lockedObject->GetTransform().SetOrientation(goatRealRotation);
	}

	if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::V)) {
		Vector3 spawnOffset1 = lockedObject->GetTransform().GetPosition() + lockedObject->GetTransform().GetOrientation() * Vector3(-4, 5, -9.75);
		Vector3 spawnOffset2 = lockedObject->GetTransform().GetPosition() + lockedObject->GetTransform().GetOrientation() * Vector3(4, 5, -10);
		GameObject* OBB1 = AddOBBToWorld(spawnOffset1, Vector3(1, 1, 1));
		OBB1->GetTransform().SetOrientation(lockedObject->GetTransform().GetOrientation());
		
		//GameObject* OBB2 = AddOBBToWorld(spawnOffset2, Vector3(1, 1, 1));
		GameObject* OBB2 = AddSphereToWorld(spawnOffset2, 1.0f);

		OBB1->GetPhysicsObject()->ApplyLinearImpulse(lockedObject->GetTransform().GetOrientation() * Vector3(5, 0, 0));
		OBB2->GetPhysicsObject()->ApplyLinearImpulse(lockedObject->GetTransform().GetOrientation() * Vector3(-5, 0, 0));
	}

	//Goat launch
	if (Window::GetMouse()->ButtonPressed(NCL::MouseButtons::LEFT)) {

		RayCollision launchCollision;
		Ray r = Ray(lockedObject->GetTransform().GetPosition() + lockedObject->GetTransform().GetOrientation() * Vector3(0, 0, -1), lockedObject->GetTransform().GetOrientation() * Vector3(0, 0, -1));

		if (world->Raycast(r, launchCollision, true)) {
			float distance = launchCollision.rayDistance;
			GameObject* target = (GameObject*)launchCollision.node;
			if (distance < 10 && target->GetRayLevel() != 40) {
				target->GetPhysicsObject()->AddForceAtPosition(r.GetDirection() * 300, launchCollision.collidedAt);
			}
		}
	}
	//Goat grab
	if (Window::GetMouse()->ButtonPressed(NCL::MouseButtons::RIGHT)) {
		if (grabObject == nullptr) {
			RayCollision launchCollision;
			Ray r = Ray(lockedObject->GetTransform().GetPosition() + lockedObject->GetTransform().GetOrientation() * Vector3(0, 0, -1), lockedObject->GetTransform().GetOrientation() * Vector3(0, 0, -1));

			//Debug::DrawLine(lockedObject->GetTransform().GetPosition(), lockedObject->GetTransform().GetPosition() + lockedObject->GetTransform().GetOrientation() * Vector3(0, 0, -10), Vector4(1, 0, 0, 1), 5);

			if (world->Raycast(r, launchCollision, true)) {
				float distance = launchCollision.rayDistance;
				GameObject* target = (GameObject*)launchCollision.node;
				if (distance < 10 && target->GetPhysicsObject()->GetInverseMass() != 0 && target->GetRayLevel() != 666) {
					grabObject = target;
					grabObject->GetRenderObject()->SetColour(Vector4(0, 0, 1, 1));
					goatConstraint = new PositionConstraint(lockedObject, grabObject, 5);
					world->AddConstraint(goatConstraint);
				}
			}
		}
		else {
			grabObject->GetRenderObject()->SetColour(grabObject->getDefaultColour());
			world->RemoveConstraint(goatConstraint, true);
			grabObject = nullptr;
			goatConstraint = nullptr;
		}
	}
}

Matrix3 TutorialGame::GoatRotationMatrix(const Vector3& direction) {
	Vector3 up = Vector3(0, 1, 0);

	Vector3 xAxis = Vector3::Cross(up, direction);
	xAxis.Normalise();
	Vector3 yAxis = Vector3::Cross(direction, xAxis);
	yAxis.Normalise();

	Matrix3 theMatrix;
	theMatrix.SetRow(0, xAxis);
	theMatrix.SetRow(1, yAxis);
	theMatrix.SetRow(2, direction);

	return theMatrix;
}

void TutorialGame::DebugObjectMovement() {
//If we've selected an object, we can manipulate it with some key presses
	if (inSelectionMode && selectionObject) {
		//Twist the selected object!
		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::LEFT)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(-10, 0, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::RIGHT)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(10, 0, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::NUM7)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(0, 10, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::NUM8)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(0, -10, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::RIGHT)) {
			selectionObject->GetPhysicsObject()->AddTorque(Vector3(10, 0, 0));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::UP)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, 0, -10));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::DOWN)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, 0, 10));
		}

		if (Window::GetKeyboard()->KeyDown(KeyboardKeys::NUM5)) {
			selectionObject->GetPhysicsObject()->AddForce(Vector3(0, -10, 0));
		}
	}
}

void TutorialGame::InitCamera() {
	world->GetMainCamera()->SetNearPlane(0.1f);
	world->GetMainCamera()->SetFarPlane(500.0f);
	world->GetMainCamera()->SetPitch(-15.0f);
	world->GetMainCamera()->SetYaw(315.0f);
	world->GetMainCamera()->SetPosition(Vector3(-60, 40, 60));
	lockedObject = nullptr;
}

void TutorialGame::InitWorld() {
	world->ClearAndErase();
	physics->Clear();

	itemCounter = 0;
	gameTime = 0;
	gameScore = 0;

	//InitMixedGridWorld(15, 15, 3.5f, 3.5f);
	GooseBehaviourTree();
	InitGameWorld("TestGrid1.txt");
	InitGameWorld2("TestGrid2.txt");

	lockedObject = AddPlayerToWorld(Vector3(30, 3, 30));
	GOOSE		 = AddGooseToWorld(Vector3(360, 5, 360));

	goatConstraint = nullptr;

	AddTargetCubeToWorld(Vector3(45, 4.5, 45), Vector3(2, 3, 2));
	AddTargetCubeToWorld(Vector3(50, 4, 45), Vector3(2, 2, 2));

	testStateObject = AddStateObjectToWorld(Vector3(150, 4, 35));
}

void TutorialGame::InitGameWorld(const std::string& filename) {

	const char WALL_NODE = 'x';
	const char DOOR_NODE = 'D';
	const char LOOT_NODE_1 = 'Q';
	const char LOOT_NODE_2 = 'W';
	const char LOOT_NODE_3 = 'E';
	const char BARRICADE_NODE_1 = 'B';
	const char BARRICADE_NODE_2 = 'N';
	const char KEY_NODE = 'K';
	const char FLOOR_NODE = '.';
	const char ALARM_NODE = 'A';
	const char CRATE_NODE_1 = 'C';
	const char CRATE_NODE_2 = 'V';
	const char SPHERE_NODE = 'S';

	std::ifstream infile(Assets::DATADIR + filename);

	float slotSize;
	int gridWidth;
	int gridHeight;

	infile >> slotSize;
	infile >> gridWidth;
	infile >> gridHeight;

	float dimensionSize;

	dimensionSize = slotSize / 2;

	AddCustomFloorToWorld(Vector3(gridWidth * slotSize, 0, gridHeight * slotSize), Vector3(slotSize*gridWidth, 2, slotSize * gridHeight));
	
	NavigationGrid worldGrid = NavigationGrid("TestGrid3.txt");
	GridNode* allNodes = worldGrid.AllNodes();
	for (int y = 0; y < gridHeight; ++y) {
		for (int x = 0; x < gridWidth; ++x) {
			GridNode& n = allNodes[(gridWidth * y) + x];
			Vector3 nodePosition = n.position;

			nodePosition = Vector3(nodePosition.x, 3, nodePosition.z);
			Vector3 wallPosition = Vector3(nodePosition.x, dimensionSize, nodePosition.z);

			char type;
			infile >> type;

			if (type == WALL_NODE) {
				AddWallToWorld(wallPosition, Vector3(dimensionSize, dimensionSize, dimensionSize));
			}
			if (type == DOOR_NODE) {
				AddDoorToWorld(wallPosition, Vector3(dimensionSize, dimensionSize, dimensionSize));
			}

			if (type == LOOT_NODE_1) {				
				PlaceLoot1(nodePosition);
			}
			if (type == LOOT_NODE_2) {
				PlaceLoot2(nodePosition);
			}
			if (type == LOOT_NODE_3) {
				PlaceLoot3(nodePosition);
			}

			if (type == BARRICADE_NODE_1) {
				PlaceBarricade(nodePosition);
			}
			if (type == BARRICADE_NODE_2) {
				PlaceBarricade2(nodePosition);
			}

			if (type == CRATE_NODE_1) {
				PlaceCrates(nodePosition);
			}
			if (type == CRATE_NODE_2) {
				PlaceCrates2(nodePosition);
			}

			if (type == SPHERE_NODE) {
				AddSphereToWorld(nodePosition, 1.0f);
			}			

			if (type == KEY_NODE) {
				AddKeyToWorld(nodePosition);
			}
			if (type == ALARM_NODE) {
				PlaceAlarm(nodePosition);
			}
		}
	}
}

void TutorialGame::PlaceAlarm(Vector3 pos) {
	AddAlarmCubeToWorld(pos + Vector3(7, 1, 7), Vector3(1, 2, 1));
	AddAlarmCubeToWorld(pos + Vector3(7, 1, -7), Vector3(1, 2, 1));
	AddAlarmCubeToWorld(pos + Vector3(-7, 1, 7), Vector3(1, 2, 1));
	AddAlarmCubeToWorld(pos + Vector3(-7, 1, -7), Vector3(1, 2, 1));
}

void TutorialGame::PlaceLoot1(Vector3 pos) {
	AddTarget10ToWorld(pos + Vector3(4, 0, 4), 1.0f);
	AddTarget10ToWorld(pos + Vector3(4, 0, -4), 1.0f);
	AddTarget10ToWorld(pos + Vector3(-4, 0, 4), 1.0f);
	AddTarget10ToWorld(pos + Vector3(-4, 0, -4), 1.0f);
	itemCounter += 4;
}

void TutorialGame::PlaceLoot2(Vector3 pos) {
	AddTarget30ToWorld(pos + Vector3(0, 1, 0), 1.0f);
	AddTarget10ToWorld(pos + Vector3(4, 0, 0), 1.0f);
	AddTarget10ToWorld(pos + Vector3(-4, 0, 0), 1.0f);
	AddTarget10ToWorld(pos + Vector3(0, 0, 4), 1.0f);
	AddTarget10ToWorld(pos + Vector3(0, 0, -4), 1.0f);
	itemCounter += 5;
}

void TutorialGame::PlaceLoot3(Vector3 pos) {
	AddTarget50ToWorld(pos + Vector3(0, 1, 0), 2.0f);
	AddTarget30ToWorld(pos + Vector3(4, 1, 0), 1.0f);
	AddTarget30ToWorld(pos + Vector3(-4, 1, 0), 1.0f);
	AddTarget30ToWorld(pos + Vector3(0, 1, 4), 1.0f);
	AddTarget30ToWorld(pos + Vector3(0, 1, -4), 1.0f);
	itemCounter += 5;
}

void TutorialGame::PlaceBarricade(Vector3 pos) {
	AddCubeToWorld(pos + Vector3(0.5, 0, 0), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(1.5, 0, 0), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(2.5, 0, 0), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(3.5, 0, 0), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(4.5, 0, 0), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(-0.5, 0, 0), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(-1.5, 0, 0), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(-2.5, 0, 0), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(-3.5, 0, 0), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(-4.5, 0, 0), Vector3(1, 1, 1));

	AddCubeToWorld(pos + Vector3(0, 2, 0), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(1, 2, 0), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(2, 2, 0), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(3, 2, 0), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(4, 2, 0), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(-1, 2, 0), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(-2, 2, 0), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(-3, 2, 0), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(-4, 2, 0), Vector3(1, 1, 1));
}

void TutorialGame::PlaceBarricade2(Vector3 pos) {
	AddCubeToWorld(pos + Vector3(0, 0, 0.5), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(0, 0, 1.5), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(0, 0, 2.5), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(0, 0, 3.5), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(0, 0, 4.5), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(0, 0, -0.5), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(0, 0, -1.5), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(0, 0, -2.5), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(0, 0, -3.5), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(0, 0, -4.5), Vector3(1, 1, 1));

	AddCubeToWorld(pos + Vector3(0, 2, 0), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(0, 2, 1), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(0, 2, 2), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(0, 2, 3), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(0, 2, 4), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(0, 2, -1), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(0, 2, -2), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(0, 2, -3), Vector3(1, 1, 1));
	AddCubeToWorld(pos + Vector3(0, 2, -4), Vector3(1, 1, 1));
}

void TutorialGame::PlaceCrates(Vector3 pos) {
	AddTargetCubeToWorld(pos + Vector3(0, 1, 2), Vector3(2, 2, 2));
	AddTargetCubeToWorld(pos + Vector3(0, 1, -2), Vector3(2, 2, 2));
	AddTargetCubeToWorld(pos + Vector3(4, 2, 0), Vector3(2, 4, 2));
	AddTargetCubeToWorld(pos + Vector3(2, 2, 0), Vector3(2, 4, 2));
	AddTargetCubeToWorld(pos + Vector3(-2, 2, 0), Vector3(2, 4, 2));
	AddTargetCubeToWorld(pos + Vector3(-4, 2, 0), Vector3(2, 4, 2));
}

void TutorialGame::PlaceCrates2(Vector3 pos) {
	AddTargetCubeToWorld(pos + Vector3(2, 1, 0), Vector3(2, 2, 2));
	AddTargetCubeToWorld(pos + Vector3(-2, 1, 0), Vector3(2, 2, 2));
	AddTargetCubeToWorld(pos + Vector3(0, 2, 4), Vector3(2, 4, 2));
	AddTargetCubeToWorld(pos + Vector3(0, 2, 2), Vector3(2, 4, 2));
	AddTargetCubeToWorld(pos + Vector3(0, 2, -2), Vector3(2, 4, 2));
	AddTargetCubeToWorld(pos + Vector3(0, 2, -4), Vector3(2, 4, 2));
}

void TutorialGame::InitGameWorld2(const std::string& filename) {

	const char WALL_NODE = 'x';
	const char FLOOR_NODE = '.';

	std::ifstream infile(Assets::DATADIR + filename);

	int slotSize;
	int gridWidth;
	int gridHeight;

	infile >> slotSize;
	infile >> gridWidth;
	infile >> gridHeight;

	slotSize = slotSize / 2;

	for (int y = 0; y < gridHeight; ++y) {
		for (int x = 0; x < gridWidth; ++x) {

			char type = 0;
			infile >> type;

			if (type == WALL_NODE) {
				AddWall2ToWorld(Vector3((2 * x) * slotSize, slotSize*3, (2 * y) * slotSize), Vector3(slotSize, slotSize, slotSize));
			}
		}
	}
}

/*

A single function to add a large immoveable cube to the bottom of our world

*/
GameObject* TutorialGame::AddFloorToWorld(const Vector3& position) {
	GameObject* floor = new GameObject();

	Vector3 floorSize = Vector3(200, 2, 200);
	AABBVolume* volume = new AABBVolume(floorSize);
	floor->SetBoundingVolume((CollisionVolume*)volume);
	floor->GetTransform()
		.SetScale(floorSize * 2)
		.SetPosition(position);

	floor->SetRenderObject(new RenderObject(&floor->GetTransform(), cubeMesh, basicTex, basicShader));
	floor->SetPhysicsObject(new PhysicsObject(&floor->GetTransform(), floor->GetBoundingVolume()));

	floor->GetPhysicsObject()->SetInverseMass(0);
	floor->GetPhysicsObject()->InitCubeInertia();

	world->AddGameObject(floor);

	return floor;
}

GameObject* TutorialGame::AddCustomFloorToWorld(const Vector3& position, const Vector3& floorSize) {
	GameObject* floor = new GameObject();

	//Vector3 floorSize = Vector3(200, 2, 200);
	AABBVolume* volume = new AABBVolume(floorSize);
	floor->SetBoundingVolume((CollisionVolume*)volume);
	floor->GetTransform()
		.SetScale(floorSize * 2)
		.SetPosition(position);

	floor->SetRenderObject(new RenderObject(&floor->GetTransform(), cubeMesh, basicTex, basicShader));
	floor->SetPhysicsObject(new PhysicsObject(&floor->GetTransform(), floor->GetBoundingVolume()));

	floor->GetPhysicsObject()->SetInverseMass(0);
	floor->GetPhysicsObject()->InitCubeInertia();

	floor->setRestitution(0.8f);

	floor->SetRayLevel(11);

	world->AddGameObject(floor);

	return floor;
}

/*

Builds a game object that uses a sphere mesh for its graphics, and a bounding sphere for its
rigid body representation. This and the cube function will let you build a lot of 'simple' 
physics worlds. You'll probably need another function for the creation of OBB cubes too.

*/
GameObject* TutorialGame::AddSphereToWorld(const Vector3& position, float radius, float inverseMass) {
	GameObject* sphere = new GameObject();

	Vector3 sphereSize = Vector3(radius, radius, radius);
	SphereVolume* volume = new SphereVolume(radius);
	sphere->SetBoundingVolume((CollisionVolume*)volume);

	sphere->GetTransform()
		.SetScale(sphereSize)
		.SetPosition(position);

	sphere->SetRenderObject(new RenderObject(&sphere->GetTransform(), sphereMesh, basicTex, basicShader));
	sphere->SetPhysicsObject(new PhysicsObject(&sphere->GetTransform(), sphere->GetBoundingVolume()));

	sphere->GetPhysicsObject()->SetInverseMass(inverseMass);
	sphere->GetPhysicsObject()->InitSphereInertia();

	sphere->setRestitution(0.8f);

	sphere->SetRayLevel(1);

	sphere->setDefaultColour(Vector4(1, 1, 1, 1));

	world->AddGameObject(sphere);

	return sphere;
}

GameObject* TutorialGame::AddTarget10ToWorld(const Vector3& position, float radius, float inverseMass) {
	GameObject* sphere = new GameObject();

	Vector3 sphereSize = Vector3(radius, radius, radius);
	SphereVolume* volume = new SphereVolume(radius);
	sphere->SetBoundingVolume((CollisionVolume*)volume);

	sphere->GetTransform()
		.SetScale(sphereSize)
		.SetPosition(position);

	sphere->SetRenderObject(new RenderObject(&sphere->GetTransform(), sphereMesh, basicTex, basicShader));
	sphere->SetPhysicsObject(new PhysicsObject(&sphere->GetTransform(), sphere->GetBoundingVolume()));

	sphere->GetPhysicsObject()->SetInverseMass(inverseMass);
	sphere->GetPhysicsObject()->InitSphereInertia();

	sphere->setRestitution(0.8f);

	sphere->SetRayLevel(10);

	sphere->GetRenderObject()->SetColour(Vector4(0.2, 1, 0.2, 0.5));
	sphere->setDefaultColour(Vector4(0.2, 1, 0.2, 0.5));

	world->AddGameObject(sphere);

	return sphere;
}

GameObject* TutorialGame::AddTarget30ToWorld(const Vector3& position, float radius, float inverseMass) {
	float meshSize = 2.0f;

	GameObject* capsule = new GameObject();

	CapsuleVolume* volume = new CapsuleVolume(0.9f * meshSize, 0.3 * meshSize);
	capsule->SetBoundingVolume((CollisionVolume*)volume);

	capsule->GetTransform()
		.SetScale(Vector3(meshSize, meshSize, meshSize))
		.SetPosition(position);

	capsule->SetRenderObject(new RenderObject(&capsule->GetTransform(), capsuleMesh, nullptr, basicShader));
	capsule->SetPhysicsObject(new PhysicsObject(&capsule->GetTransform(), capsule->GetBoundingVolume()));

	capsule->GetPhysicsObject()->SetInverseMass(inverseMass);
	capsule->GetPhysicsObject()->InitSphereInertia();

	capsule->setRestitution(0.8f);

	capsule->SetRayLevel(30);

	capsule->GetRenderObject()->SetColour(Vector4(0.8, 0.8, 0, 1));
	capsule->setDefaultColour(Vector4(0.8, 0.8, 0, 1));

	world->AddGameObject(capsule);

	return capsule;
}

GameObject* TutorialGame::AddTarget50ToWorld(const Vector3& position, float radius, float inverseMass) {
	GameObject* sphere = new GameObject();

	Vector3 sphereSize = Vector3(radius, radius, radius);
	SphereVolume* volume = new SphereVolume(radius);
	sphere->SetBoundingVolume((CollisionVolume*)volume);

	sphere->GetTransform()
		.SetScale(sphereSize)
		.SetPosition(position);

	sphere->SetRenderObject(new RenderObject(&sphere->GetTransform(), sphereMesh, basicTex, basicShader));
	sphere->SetPhysicsObject(new PhysicsObject(&sphere->GetTransform(), sphere->GetBoundingVolume()));

	sphere->GetPhysicsObject()->SetInverseMass(inverseMass);
	sphere->GetPhysicsObject()->InitSphereInertia();

	sphere->setRestitution(0.8f);

	sphere->SetRayLevel(50);

	sphere->GetRenderObject()->SetColour(Vector4(1, 0.2, 0.2, 1));
	sphere->setDefaultColour(Vector4(1, 0.2, 0.2, 1));

	world->AddGameObject(sphere);

	return sphere;
}

GameObject* TutorialGame::AddCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass) {
	GameObject* cube = new GameObject();

	AABBVolume* volume = new AABBVolume(dimensions);
	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	cube->setRestitution(0.7f);

	cube->SetRayLevel(2);

	cube->setDefaultColour(Vector4(1, 1, 1, 1));

	world->AddGameObject(cube);

	return cube;
}

GameObject* TutorialGame::AddTargetCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass) {
	GameObject* cube = new GameObject();

	AABBVolume* volume = new AABBVolume(dimensions);
	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	cube->setRestitution(0.7f);

	cube->SetRayLevel(40);

	cube->GetRenderObject()->SetColour(Vector4(0.5, 0.25, 0, 1));
	cube->setDefaultColour(Vector4(0.5, 0.25, 0, 1));

	world->AddGameObject(cube);

	return cube;
}

GameObject* TutorialGame::AddCrateToWorld(const Vector3& position, Vector3 dimensions, float inverseMass) {
	GameObject* cube = new GameObject();

	AABBVolume* volume = new AABBVolume(dimensions);
	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	cube->setRestitution(0.7f);

	cube->SetRayLevel(2);

	cube->GetRenderObject()->SetColour(Vector4(0.5, 0.25, 0, 1));
	cube->setDefaultColour(Vector4(0.5, 0.25, 0, 1));

	world->AddGameObject(cube);

	return cube;
}

GameObject* TutorialGame::AddWallToWorld(const Vector3& position, Vector3 dimensions) {
	GameObject* cube = new GameObject();

	AABBVolume* volume = new AABBVolume(dimensions);
	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(0);
	cube->GetPhysicsObject()->InitCubeInertia();

	cube->setRestitution(0.6f);

	cube->SetRayLevel(2);

	world->AddGameObject(cube);

	return cube;
}

GameObject* TutorialGame::AddWall2ToWorld(const Vector3& position, Vector3 dimensions) {
	GameObject* cube = new GameObject();

	AABBVolume* volume = new AABBVolume(dimensions);
	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(0);
	cube->GetPhysicsObject()->InitCubeInertia();

	cube->setRestitution(0.6f);

	cube->SetRayLevel(2);

	cube->GetRenderObject()->SetColour(Vector4(1, 1, 1, 0.5));

	world->AddGameObject(cube);

	return cube;
}

GameObject* TutorialGame::AddDoorToWorld(const Vector3& position, Vector3 dimensions) {
	GameObject* cube = new GameObject();

	AABBVolume* volume = new AABBVolume(dimensions);
	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(0);
	cube->GetPhysicsObject()->InitCubeInertia();

	cube->setRestitution(0.6f);

	cube->SetRayLevel(8);

	cube->GetRenderObject()->SetColour(Vector4(1, 0.7, 0.2, 1));

	world->AddGameObject(cube);

	return cube;
}

GameObject* TutorialGame::AddKeyToWorld(const Vector3& position, Vector3 dimensions, float inverseMass) {
	GameObject* cube = new GameObject();

	AABBVolume* volume = new AABBVolume(dimensions);
	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	cube->setRestitution(0.7f);

	cube->SetRayLevel(9);

	cube->setDefaultColour(Vector4(1, 0.7, 0.2, 1));

	cube->GetRenderObject()->SetColour(Vector4(1, 0.7, 0.2, 1));

	world->AddGameObject(cube);

	return cube;
}

GameObject* TutorialGame::AddAlarmCubeToWorld(const Vector3& position, Vector3 dimensions, float inverseMass) {
	GameObject* cube = new GameObject();

	AABBVolume* volume = new AABBVolume(dimensions);
	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	cube->setRestitution(0.5f);

	cube->SetRayLevel(45);

	cube->GetRenderObject()->SetColour(Vector4(0.8, 0.4, 0.8, 1));
	cube->setDefaultColour(Vector4(0.8, 0.4, 0.8, 1));

	world->AddGameObject(cube);

	return cube;
}

GameObject* TutorialGame::AddPlayerToWorld(const Vector3& position) {
	float meshSize		= 1.0f;
	float inverseMass	= 0.5f;

	GameObject* character = new GameObject();
	SphereVolume* volume  = new SphereVolume(1.0f);

	character->SetBoundingVolume((CollisionVolume*)volume);

	character->GetTransform()
		.SetScale(Vector3(meshSize, meshSize, meshSize))
		.SetPosition(position);

	character->SetRenderObject(new RenderObject(&character->GetTransform(), charMesh, nullptr, basicShader));
	character->SetPhysicsObject(new PhysicsObject(&character->GetTransform(), character->GetBoundingVolume()));

	character->GetPhysicsObject()->SetInverseMass(inverseMass);
	character->GetPhysicsObject()->InitSphereInertia();

	character->setRestitution(0.8f);

	character->SetRayLevel(6);

	character->GetRenderObject()->SetColour(Vector4(0.2, 0.2, 1, 1));

	world->AddGameObject(character);

	return character;
}

GameObject* TutorialGame::AddGooseToWorld(const Vector3& position) {
	float meshSize = 3.0f;
	float inverseMass = 0.4f;

	GameObject* character = new GameObject();
	SphereVolume* volume = new SphereVolume(3.0f);

	character->SetBoundingVolume((CollisionVolume*)volume);

	character->GetTransform()
		.SetScale(Vector3(meshSize, meshSize, meshSize))
		.SetPosition(position);

	character->SetRenderObject(new RenderObject(&character->GetTransform(), gooseMesh, nullptr, basicShader));
	character->SetPhysicsObject(new PhysicsObject(&character->GetTransform(), character->GetBoundingVolume()));

	character->GetPhysicsObject()->SetInverseMass(inverseMass);
	character->GetPhysicsObject()->InitSphereInertia();

	character->setRestitution(0.8f);

	character->SetRayLevel(666);

	character->GetRenderObject()->SetColour(Vector4(1.0, 0.2, 0.2, 1));

	world->AddGameObject(character);

	return character;
}

GameObject* TutorialGame::AddEnemyToWorld(const Vector3& position) {
	float meshSize		= 3.0f;
	float inverseMass	= 0.5f;

	GameObject* character = new GameObject();

	CapsuleVolume* volume = new CapsuleVolume(0.9f * meshSize, 0.3*meshSize);
	character->SetBoundingVolume((CollisionVolume*)volume);

	character->GetTransform()
		.SetScale(Vector3(meshSize, meshSize, meshSize))
		.SetPosition(position);

	character->SetRenderObject(new RenderObject(&character->GetTransform(), enemyMesh, nullptr, basicShader));
	character->SetPhysicsObject(new PhysicsObject(&character->GetTransform(), character->GetBoundingVolume()));

	character->GetPhysicsObject()->SetInverseMass(inverseMass);
	character->GetPhysicsObject()->InitSphereInertia();
	//character->GetPhysicsObject()->InitCubeInertia();
	world->AddGameObject(character);

	return character;
}

GameObject* TutorialGame::AddBonusToWorld(const Vector3& position) {
	GameObject* apple = new GameObject();

	SphereVolume* volume = new SphereVolume(0.5f);
	apple->SetBoundingVolume((CollisionVolume*)volume);
	apple->GetTransform()
		.SetScale(Vector3(2, 2, 2))
		.SetPosition(position);

	apple->SetRenderObject(new RenderObject(&apple->GetTransform(), sphereMesh, nullptr, basicShader));
	apple->SetPhysicsObject(new PhysicsObject(&apple->GetTransform(), apple->GetBoundingVolume()));

	apple->GetPhysicsObject()->SetInverseMass(1.0f);
	apple->GetPhysicsObject()->InitSphereInertia();

	world->AddGameObject(apple);

	return apple;
}

GameObject* TutorialGame::AddOBBToWorld(const Vector3& position, Vector3 dimensions, float inverseMass) {
	GameObject* cube = new GameObject();

	OBBVolume* volume = new OBBVolume(dimensions);
	cube->SetBoundingVolume((CollisionVolume*)volume);

	cube->GetTransform()
		.SetPosition(position)
		.SetScale(dimensions * 2);

	cube->SetRenderObject(new RenderObject(&cube->GetTransform(), cubeMesh, basicTex, basicShader));
	cube->SetPhysicsObject(new PhysicsObject(&cube->GetTransform(), cube->GetBoundingVolume()));

	cube->GetPhysicsObject()->SetInverseMass(inverseMass);
	cube->GetPhysicsObject()->InitCubeInertia();

	cube->setRestitution(0.7f);

	cube->SetRayLevel(2);

	cube->setDefaultColour(Vector4(1, 1, 1, 1));

	world->AddGameObject(cube);

	return cube;
}

StateGameObject* TutorialGame::AddStateObjectToWorld(const Vector3& position) {
	float meshSize = 3.0f;
	float inverseMass = 0.5f;

	StateGameObject* character = new StateGameObject();

	CapsuleVolume* volume = new CapsuleVolume(0.9f * meshSize, 0.3 * meshSize);
	character->SetBoundingVolume((CollisionVolume*)volume);

	character->GetTransform()
		.SetScale(Vector3(meshSize, meshSize, meshSize))
		.SetPosition(position);

	character->SetRenderObject(new RenderObject(&character->GetTransform(), enemyMesh, nullptr, basicShader));
	character->SetPhysicsObject(new PhysicsObject(&character->GetTransform(), character->GetBoundingVolume()));

	character->GetPhysicsObject()->SetInverseMass(inverseMass);
	character->GetPhysicsObject()->InitSphereInertia();

	character->GetRenderObject()->SetColour(Vector4(0.6, 0.6, 0, 1));

	character->setDefaultColour(Vector4(0.6, 0.6, 0, 1));

	character->SetRayLevel(15);

	world->AddGameObject(character);

	return character;
}

void TutorialGame::InitDefaultFloor() {
	AddFloorToWorld(Vector3(0, -20, 0));
}

void TutorialGame::InitGameExamples() {
	//AddPlayerToWorld(Vector3(0, 5, 0));
	AddEnemyToWorld(Vector3(5, 5, 0));
	AddEnemyToWorld(Vector3(15, 5, 0));
	AddBonusToWorld(Vector3(10, 5, 0));
}

void TutorialGame::InitSphereGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, float radius) {
	for (int x = 0; x < numCols; ++x) {
		for (int z = 0; z < numRows; ++z) {
			Vector3 position = Vector3(x * colSpacing, 10.0f, z * rowSpacing);
			AddSphereToWorld(position, radius, 1.0f);
		}
	}
	AddFloorToWorld(Vector3(0, -2, 0));
}

void TutorialGame::InitMixedGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing) {
	float sphereRadius = 1.0f;
	Vector3 cubeDims = Vector3(1, 1, 1);

	for (int x = 0; x < numCols; ++x) {
		for (int z = 0; z < numRows; ++z) {
			Vector3 position = Vector3(x * colSpacing, 10.0f, z * rowSpacing);

			if (rand() % 2) {
				AddCubeToWorld(position, cubeDims);
			}
			else {
				AddSphereToWorld(position, sphereRadius);
			}
		}
	}
}

void TutorialGame::InitCubeGridWorld(int numRows, int numCols, float rowSpacing, float colSpacing, const Vector3& cubeDims) {
	for (int x = 1; x < numCols+1; ++x) {
		for (int z = 1; z < numRows+1; ++z) {
			Vector3 position = Vector3(x * colSpacing, 10.0f, z * rowSpacing);
			AddCubeToWorld(position, cubeDims, 1.0f);
		}
	}
}

// Constraint Bridge Test
void TutorialGame::BridgeConstraintTest() {
	Vector3 cubeSize = Vector3(8, 8, 8);

	float invCubeMass = 5; //how heavy the middle pieces are
	int numLinks = 10;
	float maxDistance = 30; // constraint distance
	float cubeDistance = 20; // distance between links

	Vector3 startPos = Vector3(10, 100, 10);

	GameObject* start = AddCubeToWorld(startPos + Vector3(0, 0, 0), cubeSize, 0);
	GameObject* end = AddCubeToWorld(startPos + Vector3((numLinks + 2) * cubeDistance, 0, 0), cubeSize, 0);

	GameObject* previous = start;

	for (int i = 0; i < numLinks; ++i) {
		GameObject* block = AddCubeToWorld(startPos + Vector3((i + 1) * cubeDistance, 0, 0), cubeSize, invCubeMass);
		PositionConstraint* constraint = new PositionConstraint(previous, block, maxDistance);
		world->AddConstraint(constraint);
		previous = block;
	}
	PositionConstraint* constraint = new PositionConstraint(previous, end, maxDistance);
	world->AddConstraint(constraint);
}

/*
Every frame, this code will let you perform a raycast, to see if there's an object
underneath the cursor, and if so 'select it' into a pointer, so that it can be 
manipulated later. Pressing Q will let you toggle between this behaviour and instead
letting you move the camera around. 

*/
bool TutorialGame::SelectObject() {
	return false;
}

/*
If an object has been clicked, it can be pushed with the right mouse button, by an amount
determined by the scroll wheel. In the first tutorial this won't do anything, as we haven't
added linear motion into our physics system. After the second tutorial, objects will move in a straight
line - after the third, they'll be able to twist under torque aswell.
*/

void TutorialGame::MoveSelectedObject() {
	
}	


void TutorialGame::GooseBehaviourTree() {

	BehaviourAction* noiseCheck = new BehaviourAction("Noise check", [&](float dt, BehaviourState state)->BehaviourState {
		//std::cout << "running noise check\n";
		if (state == Initialise) {
			if (lockedObject->getHadCollision()) {
				lockedObject->setHadCollision(false);
				investigateTarget = lockedObject->getcollisionArea();
				state = Success;
			}
			else {
				state = Failure;
			}
		}
		return state;
		}
	);

	BehaviourAction* investigateMove = new BehaviourAction("investigate move", [&](float dt, BehaviourState state)->BehaviourState {
		//std::cout << "running investigate move\n";
		if (GooseGoatCheck()) {
			return Success;
		}
		else if ( (GOOSE->GetTransform().GetPosition() - investigateTarget ).Length() < 5 ) {
			state = Failure;
		}
		else if (state == Initialise) {
			gooseMoveToNode(investigateTarget);
			state = Ongoing;
		}
		else if (state == Ongoing) {
			gooseMoveToNode(investigateTarget);
			state = Ongoing;
		}
		return state;
		}
	);

	BehaviourAction* patrolMove = new BehaviourAction("patrol move", [&](float dt, BehaviourState state)->BehaviourState {
		//std::cout << "running patrol move\n";
		if (GooseGoatCheck()) {
			state = Success;
		}
		
		else if (state == Initialise) {
			//std::cout << "running patrol initialise\n";
			srand(time(0));
			int i = rand() % 4;
			patrolTarget = PatrolPath[i];
			gooseMoveToNode(patrolTarget);
			state = Ongoing;
		}
		else if ((GOOSE->GetTransform().GetPosition() - patrolTarget).Length() < 5) {
			//std::cout << "running patrol failure\n";
			state = Failure;
		}
		else if (state == Ongoing) {
			//std::cout << "running patrol ongoing\n";
			//std::cout << patrolTarget << ", " << GOOSE->GetTransform().GetPosition() << "\n";
			gooseMoveToNode(patrolTarget);
			state = Ongoing;
		}
		return state;
		}
	);

	BehaviourAction* chaseMove = new BehaviourAction("chase move", [&](float dt, BehaviourState state)->BehaviourState {
		//std::cout << "running chase move\n";
		if ((GOOSE->GetTransform().GetPosition() - lockedObject->GetTransform().GetPosition()).Length() < 10 || GOOSE->getGooseGotMe()) {
			state = Success;
		}
		else if (state == Initialise) {
			//std::cout << "running chase initialise\n";
			cooldownTimer = 0.0f;
			gooseMoveToNode(lockedObject->GetTransform().GetPosition());
			state = Ongoing;
		}
		else if (state == Ongoing) {
			//std::cout << "running chase ongoing\n";
			gooseMoveToNode(lockedObject->GetTransform().GetPosition());
			if (GooseGoatCheck()) {
				cooldownTimer = 0.0f;
			}
			else {
				cooldownTimer += dt;
			}
			if (cooldownTimer > 10) {
				state = Failure;
			}
			else {
				state = Ongoing;
			}
		}
		return state;
		}
	);

	BehaviourAction* attackAction = new BehaviourAction("attack action", [&](float dt, BehaviourState state)->BehaviourState {
		//std::cout << "running attack action\n";
		if (GOOSE->getGooseGotMe()) {
			std::cout << "ow, it got me!\n";
			GOOSE->setGooseGotMe(false);
			gooseNavTarget = false;
			state = Success;
		}
		else if (state == Initialise) {
			cooldownTimer = 0.0f;
			Vector3 attackVector = (lockedObject->GetTransform().GetPosition() - GOOSE->GetTransform().GetPosition()).Normalised();
			GOOSE->GetPhysicsObject()->ApplyLinearImpulse(Vector3(attackVector.x*60, attackVector.y*60, attackVector.z*60));
			state = Ongoing;
		}
		else if (state == Ongoing) {
			cooldownTimer += dt;
			if (cooldownTimer > 3) {
				gooseNavTarget = false;
				state = Failure;
			}
			else {
				state = Ongoing;
			}
		}
		return state;
		}
	);

	

	BehaviourSequence* investigate = new BehaviourSequence("Investigate Sequence");
	investigate->AddChild(noiseCheck);
	investigate->AddChild(investigateMove);

	BehaviourSequence* patrol = new BehaviourSequence("Patrol Sequence");
	patrol->AddChild(patrolMove);

	BehaviourSequence* chase = new BehaviourSequence("Chase Sequence");
	chase->AddChild(chaseMove);

	BehaviourSequence* attack = new BehaviourSequence("Attack Sequence");
	attack->AddChild(attackAction);

	BehaviourSelector* search = new BehaviourSelector("search check");
	search->AddChild(investigate);
	search->AddChild(patrol);

	rootSequence = new BehaviourSequence("Root Sequence");
	rootSequence->AddChild(search);
	rootSequence->AddChild(chase);
	rootSequence->AddChild(attack);


}

void TutorialGame::gooseMoveToNode(Vector3 destination) {
	if (!gooseNavTarget) {
		moveTarget = TestPathfinding(destination);
		gooseNavTarget = true;
	}
	if ((GOOSE->GetTransform().GetPosition() - moveTarget).Length() < 8.0f) {
		gooseNavTarget = false;
		return;
	}
	float movementx = moveTarget.x > GOOSE->GetTransform().GetPosition().x ? 20 : -20;
	float movementy = moveTarget.z > GOOSE->GetTransform().GetPosition().z ? 20 : -20;
	//std::cout << moveTarget << "\n";
	//std::cout << GetTransform().GetPosition() << "\n";
	GOOSE->GetPhysicsObject()->AddForce({ movementx, 0, movementy });
}

Vector3 TutorialGame::TestPathfinding(Vector3 destination) {
	NavigationGrid grid("TestGrid3.txt");

	NavigationPath outPath;

	bool found = grid.FindPath(GOOSE->GetTransform().GetPosition() + Vector3(0, 0, 0), destination, outPath);

	int isIt = 0;
	Vector3 pos;
	vector <Vector3 > testNodes;
	while (outPath.PopWaypoint(pos)) {
		testNodes.push_back(pos);
		isIt++;
	}

	if ((GOOSE->GetTransform().GetPosition() - destination).Length() > 5 && isIt > 1) {
		return testNodes[1];
		if ((GOOSE->GetTransform().GetPosition() - testNodes[0]).Length() > 5) {
			return testNodes[0];
		}
		else {
			return testNodes[1];
		}
	}
	return destination;
}

bool TutorialGame::GooseGoatCheck() {
	RayCollision gooseSightCollision;
	Vector3 direction = (lockedObject->GetTransform().GetPosition() - GOOSE->GetTransform().GetPosition()).Normalised();
	Ray r = Ray(GOOSE->GetTransform().GetPosition() + Vector3(direction.x*3, direction.y*3, direction.z*3), direction);

	if (world->Raycast(r, gooseSightCollision, true, selectionObject)) {
		float distance = gooseSightCollision.rayDistance;
		GameObject* target = (GameObject*)gooseSightCollision.node;
		if (distance < 50 && target == lockedObject) {
			return true;
			Debug::Print("I SEE YOU", Vector2(3, 20));
		}
	}
	return false;
}

