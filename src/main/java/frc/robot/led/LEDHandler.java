package frc.robot.led;

import frc.robot.Constants;

public class LEDHandler {
    public enum State {
        OFF,
        IDLE,
        READY_TO_SHOOT,
        NOT_READY_TO_SHOOT,
        ALLIANCE_SHIFT_OCCURRING
    }
    private State state;

    private final TejuinoBoard tejuinoBoard = new TejuinoBoard();

    public LEDHandler() {
        this.tejuinoBoard.init(Constants.LEDConstants.TEJUINO_CAN_ID);
        this.setState(State.OFF);
    }

    public void update() {
        switch (state) {
            case OFF:
                this.tejuinoBoard.turn_off_all_leds(tejuinoBoard.LED_STRIP_0);
                break;
            case IDLE:
                this.tejuinoBoard.rainbow_effect(tejuinoBoard.LED_STRIP_0);
                break;
            case READY_TO_SHOOT:

                break;
            case NOT_READY_TO_SHOOT:

                break;
            case ALLIANCE_SHIFT_OCCURRING:

                break;
        }
    }

    public void setState(State state) {
        this.state = state;
    }
}
