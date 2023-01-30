#pragma once
#include "GameObject.h"

namespace NCL {
    namespace CSC8503 {
        class StateMachine;
        class StateGameObject : public GameObject  {
        public:
            StateGameObject();
            ~StateGameObject();

            virtual void Update(float dt);


            virtual void OnCollisionBegin(GameObject* otherObject) override {
                Vector3 speed1 = physicsObject->GetLinearVelocity();
                Vector3 speed2 = otherObject->GetPhysicsObject()->GetLinearVelocity();
                if (((speed2 - speed1).Length() > 10 || otherObject->GetRayLevel() == 6) && otherObject->GetPhysicsObject()->GetInverseMass() != 0) {
                    beenHit = true;
                    culprit = otherObject;
                    counter = 0.0f;
                }
            }

        protected:
            void moveToNode(Vector3 destination);
            Vector3 TestPathfinding(Vector3 destination);
            bool canSwitchPatrol();

            void SelfRight(float dt);
            void WaitFunc(float dt);
            void Flee(float dt);

            StateMachine* stateMachine;
            StateMachine* patrolMachine;
            StateMachine* panicMachine;

            GameObject* culprit;

            bool hasTarget;
            Vector3 moveTarget;

            float counter;

            bool beenHit;

            int flightForce = 15;

            Vector3 pos1 = Vector3(40, 3, 40);
            Vector3 pos2 = Vector3(40, 3, 350);
            Vector3 pos3 = Vector3(140, 3, 350);
            Vector3 pos4 = Vector3(350, 3, 220);
            Vector3 pos5 = Vector3(350, 3, 40);
        };
    }
}