package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.led.TejuinoBoard;

public class LEDHandler extends SubsystemBase {
    private final TejuinoBoard tejuinoBoard = new TejuinoBoard();
    private State state;

    private Timer effectTimer;

    public LEDHandler() {
        this.tejuinoBoard.init(Constants.LEDConstants.TEJUINO_CAN_ID);
        this.setState(State.OFF);
        this.effectTimer = new Timer();
    }

    @Override
    public void periodic() {
        switch (state) {
            case OFF:
                this.tejuinoBoard.turn_off_all_leds(tejuinoBoard.LED_STRIP_0);
                this.tejuinoBoard.turn_off_all_leds(tejuinoBoard.LED_STRIP_1);
                break;
            case IDLE:
                if(effectTimer.isRunning()) {
                    effectTimer.stop();
                    effectTimer.reset();
                }
                //this.tejuinoBoard.rainbow_effect(tejuinoBoard.LED_STRIP_0);
                //this.tejuinoBoard.rainbow_effect(tejuinoBoard.LED_STRIP_1);
                break;
            case READY_TO_SHOOT:
                effectTimer.start();
                this.tejuinoBoard.all_leds_green(tejuinoBoard.LED_STRIP_1);
                if(effectTimer.advanceIfElapsed(2)) this.tejuinoBoard.all_leds_blue(tejuinoBoard.LED_STRIP_1);
                effectTimer.restart();
                break;
            case NOT_READY_TO_SHOOT:

                break;
            case ALLIANCE_SHIFT_OCCURRING:

                break;
        }

        SmartDashboard.putString("LED State", this.state.toString());
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
