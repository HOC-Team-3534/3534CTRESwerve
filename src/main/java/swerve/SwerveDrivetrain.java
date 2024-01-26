package swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrivetrain extends com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain {
    private static final SendableChooser<String> centricityChooser = new SendableChooser<>();

    public enum RobotCentricity {
        FieldCentric, RobotCentric
    }

    public SwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants[] modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        centricityChooser.setDefaultOption("Field Centric", RobotCentricity.FieldCentric.toString());
        centricityChooser.addOption("Robot Centric", RobotCentricity.RobotCentric.toString());
        SmartDashboard.putData("Centricity Chooser", centricityChooser);
    }

    /**
     * Gets the kinematics of the drivetrain.
     *
     * @return Kinematics of the drivetrain
     */
    public SwerveDriveKinematics getKinematics() {
        try {
            m_stateLock.readLock().lock();

            return m_kinematics;
        } finally {
            m_stateLock.readLock().unlock();
        }
    }

    public ChassisSpeeds getChassisSpeeds() {
        return this.getKinematics().toChassisSpeeds(this.getState().ModuleStates);
    }

    public RobotCentricity getChosenRobotCentricity() {
        return centricityChooser.getSelected().equals(RobotCentricity.FieldCentric.toString())
                ? RobotCentricity.FieldCentric
                : RobotCentricity.RobotCentric;
    }

}