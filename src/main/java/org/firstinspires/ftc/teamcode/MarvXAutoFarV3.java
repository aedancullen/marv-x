import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.MarvXAutoCommonV3;

@Autonomous(name="Far")
public class MarvXAutoFarV3 extends LinearOpMode {
    MarvXAutoCommonV3 autoCommon;
    public void runOpMode() {
        autoCommon = new MarvXAutoCommonV3();
        autoCommon.runningMode = this;
        autoCommon.gamepad1 = gamepad1;
        autoCommon.gamepad2 = gamepad2;
        autoCommon.hardwareMap = hardwareMap;
        autoCommon.telemetry = telemetry;
        autoCommon.runDepot();
    }
}        