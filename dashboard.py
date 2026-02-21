import wpilib
from AutoSequencerV2.autoSequencer import AutoSequencer
from dashboardWidgets.autoChooser import AutoChooser
from dashboardWidgets.circularGauge import CircularGauge
from dashboardWidgets.swerveState import SwerveState
from dashboardWidgets.icon import Icon
from dashboardWidgets.text import Text
from utils.faults import FaultWrangler
from utils.signalLogging import addLog
from utils.units import m2ft
from webserver.webserver import Webserver
from drivetrain.controlStrategies.autoSteer import AutoSteer
from drivetrain.controlStrategies.autoDrive import AutoDrive
from fuelSystems import gameStateTracker, shooterControl
from fuelSystems import intakeControl


class Dashboard:
    def __init__(self):
        webServer = Webserver()

        #all the indicators in the middle. Top row, then bottom row
        webServer.addDashboardWidget(Icon(35, 50, "/SmartDashboard/isautoSteerState", "#9632bf", "autoSteer"))
        webServer.addDashboardWidget(Icon(45, 50, "/SmartDashboard/isRedIconState", "#FF0000", "allianceRed"))
        webServer.addDashboardWidget(Icon(55, 50, "/SmartDashboard/isBlueIconState", "#0000FF", "allianceBlue"))
        webServer.addDashboardWidget(Icon(65, 50, "/SmartDashboard/PE Vision Targets Seen", "#00AAFF", "vision"))
        webServer.addDashboardWidget(Icon(65, 65, "/SmartDashboard/faultIcon", "#FF2200", "warning"))
        webServer.addDashboardWidget(Icon(55, 65, "/SmartDashboard/isHubActiveState", "#00FF2F", "hubActive"))
        webServer.addDashboardWidget(Icon(45, 65, "/SmartDashboard/wristPosition", "#FF6607", "vacuum"))
        webServer.addDashboardWidget(Icon(35, 65, "/SmartDashboard/pieceStaged", "#B5B5B5", "algae"))

        #the fault descriptions
        webServer.addDashboardWidget(Text(50, 75, "/SmartDashboard/faultDescription"))

        #swerve states icons
        webServer.addDashboardWidget(SwerveState(85, 15))

        #auto stuff
        webServer.addDashboardWidget(
            AutoChooser(
                50,
                10,
                AutoSequencer().getDelayModeNTTableName(),
                AutoSequencer().getDelayModeList(),
            )
        )
        webServer.addDashboardWidget(
            AutoChooser(
                50,
                20,
                AutoSequencer().getFlipModeNTTableName(),
                AutoSequencer().getFlipModeList(),
            )
        )
        webServer.addDashboardWidget(
            AutoChooser(
                50,
                30,
                AutoSequencer().getMainModeNTTableName(),
                AutoSequencer().getMainModeList(),
            )
        )

        # Now, this is the stuff that updates the dashboard, through logs
        addLog("isautoSteerState",  
               lambda: (
            Icon.kON if AutoSteer().isRunning()
            else Icon.kOFF)
        )

        addLog("isRedIconState",  
               lambda: (
            Icon.kON if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed 
            else Icon.kOFF)
        )

        addLog("isBlueIconState", 
            lambda: (
            Icon.kON if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kBlue 
            else Icon.kOFF)
        )

        addLog("faultIcon",
                lambda: (Icon.kBLINK_FAST if FaultWrangler().hasActiveFaults() else Icon.kOFF)
        )

        addLog("isHubActiveState", 
            lambda: (Icon.kON if gameStateTracker.GameStateTracker().getHubActive() else Icon.kOFF)
        )

        addLog("wristPosition", 
            lambda: (Icon.kON if intakeControl.IntakeControl().getIntakeWristState() else Icon.kOFF)
        )

        addLog("pieceStaged", 
            lambda: (Icon.kON if shooterControl.ShooterController().getGamePieceStaged() else Icon.kOFF)
        )

        # Test Only.
        # TODO: Real data
        addLog("reefGoalPosIdx",
                lambda: (AutoDrive().getDashTargetPositionIndex()) #Bottom is the side facing our driver station.
        )


