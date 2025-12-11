package org.firstinspires.ftc.teamcode;

public class Toggler {

    Boolean lastInputState = false;
    Boolean currentState = false;

    Toggler() {

    }

    public boolean update(boolean inputState) {
        if (inputState && !this.lastInputState) {
            this.currentState = !this.currentState;
        }

        this.lastInputState = inputState;

        return this.currentState;
    }
}
