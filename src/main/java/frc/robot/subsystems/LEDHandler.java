package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.led.TejuinoBoard;

public class LEDHandler extends SubsystemBase {
    private final TejuinoBoard tejuinoBoard = new TejuinoBoard();
    private State state;

    public LEDHandler() {
        this.tejuinoBoard.init(Constants.LEDConstants.TEJUINO_CAN_ID);
        this.setState(State.OFF);
    }

    @Override
    public void periodic() {
        switch (state) {
            case OFF:
                this.tejuinoBoard.turn_off_all_leds(tejuinoBoard.LED_STRIP_0);
                this.tejuinoBoard.turn_off_all_leds(tejuinoBoard.LED_STRIP_1);
                break;
            case IDLE:
                this.tejuinoBoard.rainbow_effect(tejuinoBoard.LED_STRIP_0);
                this.tejuinoBoard.rainbow_effect(tejuinoBoard.LED_STRIP_1);
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

    public enum State {
        OFF,
        IDLE,
        READY_TO_SHOOT,
        NOT_READY_TO_SHOOT,
        ALLIANCE_SHIFT_OCCURRING
    }
}
