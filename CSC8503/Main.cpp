#include "Window.h"

#include "Debug.h"

#include "StateMachine.h"
#include "StateTransition.h"
#include "State.h"

#include "GameServer.h"
#include "GameClient.h"

#include "NavigationGrid.h"
#include "NavigationMesh.h"

#include "TutorialGame.h"
#include "NetworkedGame.h"

#include "PushdownMachine.h"

#include "PushdownState.h"

#include "BehaviourNode.h"
#include "BehaviourSelector.h"
#include "BehaviourSequence.h"
#include "BehaviourAction.h"

using namespace NCL;
using namespace CSC8503;

#include <chrono>
#include <thread>
#include <sstream>

vector <Vector3 > testNodes;
void TestPathfinding() {
	NavigationGrid grid("TestGrid3.txt");

	NavigationPath outPath;

	Vector3 startPos(120, 4, 20);
	Vector3 endPos(35, 3, 35);
	
	bool found = grid.FindPath(endPos, startPos, outPath);

	Vector3 pos;

	testNodes.clear();

	while (outPath.PopWaypoint(pos)) {
		testNodes.push_back(pos);
	}
}

void DisplayPathfinding() {
	//for (int i = 1; i < testNodes.size(); ++i) {
	//	Vector3 a = testNodes[i - 1];
	//	Vector3 b = testNodes[i];
	//
	//	Debug::DrawLine(a, b, Vector4(0, 1, 0, 1));
	//}
}

/*

The main function should look pretty familar to you!
We make a window, and then go into a while loop that repeatedly
runs our 'game' until we press escape. Instead of making a 'renderer'
and updating it, we instead make a whole game, and repeatedly update that,
instead. 

This time, we've added some extra functionality to the window class - we can
hide or show the 

*/

// Behaviour Tree Stuff
void TestBehaviourTree() {
	float behaviourTimer;
	float distanceToTarget;
	BehaviourAction* findKey = new BehaviourAction("Find Key", [&](float dt, BehaviourState state)->BehaviourState {
		if (state == Initialise) {
			std::cout << "Looking for a key!\n";
			behaviourTimer = rand() % 100;
			state = Ongoing;
		}
		else if (state == Ongoing) {
			behaviourTimer -= dt;
			if (behaviourTimer <= 0.0f) {
				std::cout << "Found a key!\n";
				return Success;
			}
		}
		return state; //will be ’ongoing’ until success
	}
	);

	BehaviourAction* goToRoom = new BehaviourAction("Go To Room", [&](float dt, BehaviourState state)->BehaviourState {
		if (state == Initialise) {
			std::cout << "Going to the loot room!\n";
			state = Ongoing;
		}
		else if (state == Ongoing) {
			distanceToTarget -= dt;
			if (distanceToTarget <= 0.0f) {
				std::cout << "Reached room!\n";
				return Success;
			}
		}
		return state; //will be ’ongoing’ until success
	}
	);

	BehaviourAction* openDoor = new BehaviourAction("Open Door", [&](float dt, BehaviourState state)->BehaviourState {
		if (state == Initialise) {
			std::cout << "Opening Door!\n";
			return Success;
		}
		return state;
	}
	);

	BehaviourAction* lookForTreasure = new BehaviourAction("Look For Treasure", [&](float dt, BehaviourState state)->BehaviourState {
		if (state == Initialise) {
			std::cout << "Looking for treasure !\n";
			return Ongoing;
		}
		else if (state == Ongoing) {
			bool found = rand() % 2;
			if (found) {
				std::cout << "I found some treasure !\n";
				return Success;
			}
			std::cout << "No treasure in here ...\n";
			return Failure;
		}
		return state;
	}
	);

	BehaviourAction* lookForItems = new BehaviourAction("Look For Items", [&](float dt, BehaviourState state)->BehaviourState {
		if (state == Initialise) {
			std::cout << "Looking for items!\n";
			return Ongoing;
		}
		else if (state == Ongoing) {
			bool found = rand() % 2;
			if (found) {
				std::cout << "I found some items!\n";
				return Success;
			}
			std::cout << "No items in here ...\n";
			return Failure;
		}
		return state;
	}
	);

	BehaviourSequence* sequence = new BehaviourSequence("Room Sequence");
	sequence->AddChild(findKey);
	sequence->AddChild(goToRoom);
	sequence->AddChild(openDoor);

	BehaviourSelector* selection = new BehaviourSelector("Loot Selection");
	selection->AddChild(lookForTreasure);
	selection->AddChild(lookForItems);

	BehaviourSequence* rootSequence = new BehaviourSequence("Root Sequence");
	rootSequence->AddChild(sequence);
	rootSequence->AddChild(selection);

	for (int i = 0; i < 5; ++i) {
		rootSequence->Reset();
		behaviourTimer = 0.0f;
		distanceToTarget = rand() % 250;
		BehaviourState state = Ongoing;
		std::cout << "We’re going on an adventure !\n";
		while (state == Ongoing) {
			state = rootSequence->Execute(1.0f); //fake dt
		}
		if (state == Success) {
			std::cout << "What a successful adventure !\n";
		}
		else if (state == Failure) {
			std::cout << "What a waste of time!\n";
		}
	}
	std::cout << "All done!\n";
}



Window* w = Window::CreateGameWindow("CSC8503 Game technology!", 1280, 720);

TutorialGame* g;


class PauseScreen : public PushdownState {
	PushdownResult OnUpdate(float dt, PushdownState** newState) override {
		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::U)) {
			return PushdownResult::Pop;
		}
		return PushdownResult::NoChange;
	}
	void OnAwake() override {
		std::cout << "Press U to unpause game!\n";
	}
};

class GameScreen : public PushdownState {
	PushdownResult OnUpdate(float dt, PushdownState** newState) override {
		w->GetTimer()->GetTimeDeltaSeconds(); //Clear the timer so we don't get a larget first dt!
		while (w->UpdateWindow() && !Window::GetKeyboard()->KeyDown(KeyboardKeys::ESCAPE)) {
			float dt = w->GetTimer()->GetTimeDeltaSeconds();
			if (dt > 0.1f) {
				std::cout << "Skipping large time delta" << std::endl;
				continue; //must have hit a breakpoint or something to have a 1 second frame time!
			}
			if (Window::GetKeyboard()->KeyDown(KeyboardKeys::P)) {
				*newState = new PauseScreen();
				return PushdownResult::Push;
			}
			if (Window::GetKeyboard()->KeyDown(KeyboardKeys::F1)) {
				std::cout << "Returning to main menu!\n";
				return PushdownResult::Pop;
			}
			if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::PRIOR)) {
				w->ShowConsole(true);
			}
			if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::NEXT)) {
				w->ShowConsole(false);
			}

			if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::T)) {
				w->SetWindowPosition(0, 0);
			}

			TestPathfinding();
			DisplayPathfinding();

			w->SetTitle("Gametech frame time:" + std::to_string(1000.0f * dt));

			g->UpdateGame(dt);

			if (g->getTime() > 200) {
				std::cout << "Game over!\n";
				int s = g->getEndScore();
				int i = g->getEndItems();
				if (i > 0) {
					std::cout << "You have lost!\n";
					std::cout << "Remaining items: " << i << "\n";
				}
				else {
					std::cout << "You have won!\n";
				}
				std::cout << "Your final score: " << s << "\n";
				std::cout << "Returning to menu!\n";
				return PushdownResult::Pop;
			}
		}

		return PushdownResult::NoChange;
	};
	void OnAwake() override {
		
	}
protected:

};

class IntroScreen : public PushdownState {
	PushdownResult OnUpdate(float dt, PushdownState** newState) override {
		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::SPACE)) {
			*newState = new GameScreen();
			g->InitWorld();
			return PushdownResult::Push;
		}
		if (Window::GetKeyboard()->KeyPressed(KeyboardKeys::ESCAPE)) {
			return PushdownResult::Pop;
		}
		return PushdownResult::NoChange;
	};
	void OnAwake() override {
		std::cout << "Welcome to a really awesome game!\n";
		std::cout << "Press Space To Begin or escape to quit!\n";
	}
};

void TestPushdownAutomata(Window* w) {
	PushdownMachine machine(new IntroScreen());
	while (w->UpdateWindow()) {
		float dt = w->GetTimer()->GetTimeDeltaSeconds();
		if (!machine.Update(dt)) {
			return;
		}
	}
}


int main() {
	if (!w->HasInitialised()) {
		return -1;
	}	
	w->ShowOSPointer(false);
	w->LockMouseToWindow(true);
	

	g = new TutorialGame();

	TestPushdownAutomata(w);
	
	Window::DestroyGameWindow();
}