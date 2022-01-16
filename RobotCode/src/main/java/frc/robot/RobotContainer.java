// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoPowerCellMover;
import frc.robot.commands.GroundPickUpCommand;
import frc.robot.commands.ManuallyControlClimber;
import frc.robot.commands.ManuallyControlIndexer;
import frc.robot.commands.ManuallyControlShooter;
import frc.robot.commands.PDPLogger;
import frc.robot.commands.RumbleCommand;
import frc.robot.commands.SingulatorCommand;
import frc.robot.commands.autocommands.AutoAim;
import frc.robot.commands.autocommands.AutoMoves;
import frc.robot.commands.drivebase.AutoCollect;
import frc.robot.commands.drivebase.ManuallyControlDrivebase;
import frc.robot.commands.vision.SetVisionMode;
import frc.robot.oi.JoystickAxisButton;
import frc.robot.oi.JoystickPovButton;
import frc.robot.oi.XBox360Controller;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.PowerCellMover;
import frc.robot.subsystems.Telemetry;
import frc.robot.subsystems.VisionProcessor;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    public static final OI oi = new OI();

    // Robot subsystems
    public DriveBase drivebase = new DriveBase();
    public Compressor compressor = new Compressor(Constants.PCM_NUM, PneumaticsModuleType.CTREPCM);
    public VisionProcessor vision = new VisionProcessor();
    public LimeLight limelightport = new LimeLight("port");
    public LimeLight limelightcell = new LimeLight("cell");
    // public GroundPickUp groundPickUp = new GroundPickUp();
    // public Indexer indexer = new Indexer();
    // public Singulator singulator = new Singulator();
    public PowerCellMover powerCellMover = new PowerCellMover();
    // public Shooter shooter = new Shooter();
    public Climber climber = new Climber();
    public Telemetry telemetry = new Telemetry();

    // Robot Commands
    private ManuallyControlClimber m_ManuallyControlClimber = new ManuallyControlClimber(climber);
    private ManuallyControlDrivebase m_ManuallyControlDrivebase = new ManuallyControlDrivebase(drivebase);
    private GroundPickUpCommand m_GroundPickUpCommand = new GroundPickUpCommand();
    private ManuallyControlIndexer m_ManuallyControlIndexer = new ManuallyControlIndexer();
    private AutoPowerCellMover m_AutoPowerCellMover = new AutoPowerCellMover(powerCellMover);
    private ManuallyControlShooter m_ManuallyControlShooter = new ManuallyControlShooter();
    private SingulatorCommand m_SingulatorCommand = new SingulatorCommand();
    private PDPLogger m_PdpLogger = new PDPLogger(telemetry);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Init default commands here, DEFAULT COMMANDS CANNOT END!
        climber.setDefaultCommand(m_ManuallyControlClimber);
        drivebase.setDefaultCommand(m_ManuallyControlDrivebase);
        // groundPickUp.setDefaultCommand(m_GroundPickUpCommand);
        // indexer.setDefaultCommand(new ManuallyControlIndexer());
        powerCellMover.setDefaultCommand(m_AutoPowerCellMover);
        // shooter.setDefaultCommand(m_ManuallyControlShooter);
        // singulator.setDefaultCommand(m_SingulatorCommand);
        telemetry.setDefaultCommand(m_PdpLogger);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Create some buttons
        new JoystickPovButton(oi.driverController, 0)
                .whenPressed(new SetVisionMode(vision, VisionProcessor.VisionMode.UPPER_PORT_HUMAN));
        new JoystickPovButton(oi.driverController, 90)
                .whenPressed(new SetVisionMode(vision, VisionProcessor.VisionMode.UPPER_PORT_MACHINE));
        new JoystickPovButton(oi.driverController, 180)
                .whenPressed(new SetVisionMode(vision, VisionProcessor.VisionMode.SWITCH_HUMAN));
        new JoystickPovButton(oi.driverController, 270)
                .whenPressed(new SetVisionMode(vision, VisionProcessor.VisionMode.CONTROL_PANEL_MACHINE));
        // cameraViewSwitcher.close(); // Don't need this one anymore?

        JoystickAxisButton driverRumblerLeft = new JoystickAxisButton(oi.driverController,
                XBox360Controller.Axis.LEFT_TRIGGER.Number());
        driverRumblerLeft.whenPressed(new RumbleCommand(OI.Controller.DRIVER,
                Joystick.RumbleType.kLeftRumble, 1, 1.0));

        JoystickAxisButton driverRumblerRight = new JoystickAxisButton(oi.driverController,
                XBox360Controller.Axis.RIGHT_TRIGGER.Number());
        driverRumblerRight.whenPressed(new RumbleCommand(OI.Controller.DRIVER,
                Joystick.RumbleType.kRightRumble, 1, 1.0));

        JoystickAxisButton weaponsRumblerLeft = new JoystickAxisButton(oi.weaponsController,
                XBox360Controller.Axis.LEFT_TRIGGER.Number());
        weaponsRumblerLeft.whenPressed(new RumbleCommand(OI.Controller.WEAPONS,
                Joystick.RumbleType.kLeftRumble, 1, 1.0));

        JoystickAxisButton weaponsRumblerRight = new JoystickAxisButton(oi.weaponsController,
                XBox360Controller.Axis.RIGHT_TRIGGER.Number());
        weaponsRumblerRight.whenPressed(new RumbleCommand(OI.Controller.WEAPONS,
                Joystick.RumbleType.kRightRumble, 1, 1.0));

        oi.runIndexer = new JoystickButton(oi.weaponsController, XBox360Controller.Button.B.Number());
        // groundPickUpDeploy = new JoystickButton(weaponsController,
        // XBox360Controller.Button.X.Number());

        JoystickButton visionAimRangeA = new JoystickButton(oi.driverController, OI.BUTTON_VISION_AIM_A);
        visionAimRangeA.whileHeld(new AutoAim(drivebase, limelightport, Constants.VISION_RANGE_A_INCH));

        JoystickButton visionAimRangeB = new JoystickButton(oi.driverController, OI.BUTTON_VISION_AIM_B);
        visionAimRangeB.whileHeld(new AutoAim(drivebase, limelightport, Constants.VISION_RANGE_B_INCH));

        JoystickButton visionAimRangeC = new JoystickButton(oi.driverController, OI.BUTTON_VISION_AIM_C);
        visionAimRangeC.whileHeld(new AutoAim(drivebase, limelightport, Constants.VISION_RANGE_C_INCH));

        JoystickButton visionAimRangeD = new JoystickButton(oi.driverController, OI.BUTTON_VISION_AIM_D);
        visionAimRangeD.whileHeld(new AutoAim(drivebase, limelightport, Constants.VISION_RANGE_D_INCH));

        JoystickButton autocollect = new JoystickButton(oi.driverController, OI.BUTTON_AUTO_COLLECT);
        autocollect.whileHeld(new AutoCollect(drivebase, limelightcell));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // var commandgroup = new AutoMoves();
        // CommandScheduler.getInstance().add(commandgroup);
        return new AutoMoves(drivebase);
    }
}
