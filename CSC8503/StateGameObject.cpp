#include "StateGameObject.h"
#include "StateTransition.h"
#include "StateMachine.h"
#include "State.h"
#include "PhysicsObject.h"

#include "NavigationGrid.h"
#include "NavigationMesh.h"

#include "Debug.h"

using namespace NCL;
using namespace CSC8503;

StateGameObject::StateGameObject() {
	counter = 0.0f;
	beenHit = false;
	stateMachine = new StateMachine();
	patrolMachine = new StateMachine();
	panicMachine = new StateMachine();

	// Superstates
	State* PatrolMode = new State([&](float dt)-> void {patrolMachine->Update(dt);});
	State* PanicMode = new State([&](float dt)-> void {panicMachine->Update(dt); });
	//
	// Patrol states
	State* Patrol1 = new State([&](float dt)-> void{moveToNode(pos1);});
	State* Patrol2 = new State([&](float dt)-> void{moveToNode(pos2);});
	State* Patrol3 = new State([&](float dt)-> void{moveToNode(pos3);});
	State* Patrol4 = new State([&](float dt)-> void{moveToNode(pos4);});
	State* Patrol5 = new State([&](float dt)-> void{moveToNode(pos5);});
	//
	// Panic states
	State* Run = new State([&](float dt)-> void{Flee(dt); });
	State* Wait = new State([&](float dt)-> void{WaitFunc(dt); });
	State* Upright = new State([&](float dt)-> void{SelfRight(dt); });
	//
	// State assignment
	stateMachine->AddState(PatrolMode);
	stateMachine->AddState(PanicMode);

	patrolMachine->AddState(Patrol1);
	patrolMachine->AddState(Patrol2);
	patrolMachine->AddState(Patrol3);
	patrolMachine->AddState(Patrol4);
	patrolMachine->AddState(Patrol5);

	panicMachine->AddState(Run);
	panicMachine->AddState(Wait);
	panicMachine->AddState(Upright);

	// Transition assignment
	stateMachine->AddTransition(new StateTransition(PatrolMode, PanicMode, [&]()-> bool{ return beenHit; }));
	stateMachine->AddTransition(new StateTransition(PanicMode, PatrolMode, [&]()-> bool { return canSwitchPatrol(); }));

	patrolMachine->AddTransition(new StateTransition(Patrol1, Patrol2, [&]()-> bool{ return (GetTransform().GetPosition() - pos1).Length() < 5; }));
	patrolMachine->AddTransition(new StateTransition(Patrol2, Patrol3, [&]()-> bool{ return (GetTransform().GetPosition() - pos2).Length() < 5; }));
	patrolMachine->AddTransition(new StateTransition(Patrol3, Patrol4, [&]()-> bool{ return (GetTransform().GetPosition() - pos3).Length() < 5; }));
	patrolMachine->AddTransition(new StateTransition(Patrol4, Patrol5, [&]()-> bool{ return (GetTransform().GetPosition() - pos4).Length() < 5; }));
	patrolMachine->AddTransition(new StateTransition(Patrol5, Patrol1, [&]()-> bool{ return (GetTransform().GetPosition() - pos5).Length() < 5; }));

	panicMachine->AddTransition(new StateTransition(Upright, Run, [&]()-> bool { return  beenHit; }));
	panicMachine->AddTransition(new StateTransition(Upright, Wait, [&]()-> bool { return  Quaternion::Dot(GetTransform().GetOrientation(), Quaternion()) > 0.75; }));
	panicMachine->AddTransition(new StateTransition(Wait, Upright, [&]()-> bool { return  Quaternion::Dot(GetTransform().GetOrientation(), Quaternion()) < 0.75; }));
	panicMachine->AddTransition(new StateTransition(Run, Wait, [&]()-> bool { return (GetTransform().GetPosition() - culprit->GetTransform().GetPosition()).Length() > 40; }));
	panicMachine->AddTransition(new StateTransition(Wait, Run, [&]()-> bool { return  beenHit; }));
	//

	hasTarget = false;
}

StateGameObject::~StateGameObject() {
	delete stateMachine;
	delete patrolMachine;
	delete panicMachine;
}

void StateGameObject::Update(float dt) {
	stateMachine->Update(dt);
}

bool StateGameObject::canSwitchPatrol() {
	if (counter > 15 && Quaternion::Dot(GetTransform().GetOrientation(), Quaternion()) > 0.9) {
		return true;
	}
	return false;
}

//void StateGameObject::moveToNode(Vector3 destination) {
//	Vector3 moveTarget = TestPathfinding(destination);
//
//	float movementx = moveTarget.x > GetTransform().GetPosition().x ? 10 : -10;
//	float movementy = moveTarget.z > GetTransform().GetPosition().z ? 10 : -10;
//	std::cout << moveTarget << "\n";
//	std::cout << GetTransform().GetPosition() << "\n";
//	GetPhysicsObject()->AddForce({ movementx, 0, movementy });
//}

void StateGameObject::moveToNode(Vector3 destination) {
	if (Quaternion::Dot(GetTransform().GetOrientation(), Quaternion()) < 0.75) {
		GetTransform().SetPosition(GetTransform().GetPosition() + Vector3(0, 2, 0));
		GetTransform().SetOrientation(Quaternion());
		GetPhysicsObject()->SetAngularVelocity(Vector3(0, 0, 0));
	}

	if (!hasTarget) {
		moveTarget = TestPathfinding(destination);
		hasTarget = true;
	}
	if ((GetTransform().GetPosition() - moveTarget).Length() < 5.0f) {
		hasTarget = false;
		return;
	}

	float movementx = moveTarget.x > GetTransform().GetPosition().x ? 10 : -10;
	float movementy = moveTarget.z > GetTransform().GetPosition().z ? 10 : -10;
	//std::cout << moveTarget << "\n";
	//std::cout << GetTransform().GetPosition() << "\n";
	GetPhysicsObject()->AddForce({ movementx, 0, movementy });
}

Vector3 StateGameObject::TestPathfinding(Vector3 destination) {
	NavigationGrid grid("TestGrid3.txt");

	NavigationPath outPath;

	bool found = grid.FindPath(GetTransform().GetPosition() + Vector3(0, 0, 0), destination, outPath);

	int isIt = 0;
	Vector3 pos;
	vector <Vector3 > testNodes;
	while (outPath.PopWaypoint(pos)) {
		testNodes.push_back(pos);
		isIt++;
	}

	//if (found) {
	//	for (int i = 1; i < testNodes.size(); ++i) {
	//		Vector3 a = testNodes[i - 1];
	//		Vector3 b = testNodes[i];
	//
	//		Debug::DrawLine(a, b, Vector4(0, 1, 0, 1), 1.0f);
	//	}
	//}

	//return testNodes[0];

	if ((GetTransform().GetPosition() - destination).Length() > 5 && isIt > 1) {
		if ((GetTransform().GetPosition() - testNodes[0]).Length() > 5) {
			return testNodes[0];
		}
		else {
			return testNodes[1];
		}
	}
	return destination;
}

void StateGameObject::SelfRight(float dt) {
	GetTransform().SetPosition(GetTransform().GetPosition() + Vector3(0, 2, 0));
	GetTransform().SetOrientation(Quaternion());
	GetPhysicsObject()->SetAngularVelocity(Vector3(0, 0, 0));
	counter += dt;
}

void StateGameObject::WaitFunc(float dt) {
	counter += dt;
}

void StateGameObject::Flee(float dt) {
	counter += dt;
	Vector3 flightDirection = (GetTransform().GetPosition() - culprit->GetTransform().GetPosition()).Normalised();
	GetPhysicsObject()->AddForce({ flightDirection.x* flightForce, 0, flightDirection.z * flightForce });

	if (beenHit) {
		beenHit = false;
		counter = 0.0f;
	}
}