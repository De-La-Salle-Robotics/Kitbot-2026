package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandGameSirT3Lite extends CommandGenericHID{
    /**
     * These bindings only apply if the GameSir is in D-input mode.
     * Make sure this is the case by pressing and holding Home+B for 2 seconds.
     */
    private enum Buttons {
        X(1),
        A(2),
        B(3),
        Y(4),
        LeftBumper(5),
        RightBumper(6),
        NotMain(7), // When the button is pressed, the value is false for a half second
        Capture(9),
        Menu(10),
        LPress(11),
        RPress(12),
        Home(13),
        Screenshot(14)
        ;
        private final int value;
        private Buttons(int value) {
            this.value = value;
        }
    }
    private enum Axis {
        LeftX(0),
        LeftY(1),
        RightX(2),
        RightY(5),
        LeftTrigger(3),
        RightTrigger(4)
        ;
        private final int value;
        private Axis(int value) {
            this.value = value;
        }
    }
    GenericHID m_hid;

    public CommandGameSirT3Lite(int port) {
        super(port);
        m_hid = new GenericHID(port);
    }

    public Trigger x() {
        return button(Buttons.X.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }
    public Trigger a() {
        return button(Buttons.A.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }
    public Trigger b() {
        return button(Buttons.B.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }
    public Trigger y() {
        return button(Buttons.Y.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }
    public Trigger leftBumper() {
        return button(Buttons.LeftBumper.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }
    public Trigger rightBumper() {
        return button(Buttons.RightBumper.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }
    public Trigger screenshot() {
        return button(Buttons.Screenshot.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }
    public Trigger menu() {
        return button(Buttons.Menu.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }
    public Trigger capture() {
        return button(Buttons.Capture.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }
    public Trigger home() {
        return button(Buttons.Home.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }
    public Trigger lPress() {
        return button(Buttons.LPress.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }
    public Trigger rPress() {
        return button(Buttons.RPress.value, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public double getLeftX() {
        return m_hid.getRawAxis(Axis.LeftX.value);
    }
    public double getLeftY() {
        return m_hid.getRawAxis(Axis.LeftY.value);
    }
    public double getRightX() {
        return m_hid.getRawAxis(Axis.RightX.value);
    }
    public double getRightY() {
        return m_hid.getRawAxis(Axis.RightY.value);
    }
    public double getLeftTrigger() {
        return m_hid.getRawAxis(Axis.LeftTrigger.value);
    }
    public double getRightTrigger() {
        return m_hid.getRawAxis(Axis.RightTrigger.value);
    }
    public Trigger leftTrigger(double threshold) {
        return axisGreaterThan(Axis.LeftTrigger.value, threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }
    public Trigger rightTrigger(double threshold) {
        return axisGreaterThan(Axis.RightTrigger.value, threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }
    public Trigger leftTrigger() {
        return leftTrigger(0);
    }
    public Trigger rightTrigger() {
        return rightTrigger(0);
    }
}
