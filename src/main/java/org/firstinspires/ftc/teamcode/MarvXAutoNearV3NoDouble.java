import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.MarvXAutoCommonV3;

@Autonomous(name="Near, No Double")
public class MarvXAutoNearV3NoDouble extends LinearOpMode {
    MarvXAutoCommonV3 autoCommon;
    public void runOpMode() {
        autoCommon = new MarvXAutoCommonV3();
        autoCommon.gamepad1 = gamepad1;
        autoCommon.gamepad2 = gamepad2;
        autoCommon.runningMode = this;
        autoCommon.hardwareMap = hardwareMap;
        autoCommon.telemetry = telemetry;
        autoCommon.runCrater();
    }
}        