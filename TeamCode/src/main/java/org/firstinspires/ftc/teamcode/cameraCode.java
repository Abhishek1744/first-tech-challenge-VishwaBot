package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "HuskyLens Dashboard Test", group = "Test")
public class cameraCode extends OpMode {

    private HuskyLens huskyLens;
    private FtcDashboard dashboard;

    @Override
    public void init() {
        huskyLens = hardwareMap.get(HuskyLens.class, "husky");
        dashboard = FtcDashboard.getInstance();

        telemetry.addLine("HuskyLens Initialized");
        telemetry.addLine("Open FTC Dashboard");
        telemetry.update();
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        HuskyLens.Block[] blocks = huskyLens.blocks();

        if (blocks.length > 0) {
            HuskyLens.Block target = blocks[0];

            packet.put("Detected", true);
            packet.put("ID", target.id);
            packet.put("X", target.x);
            packet.put("Y", target.y);
            packet.put("Width", target.width);
            packet.put("Height", target.height);

            telemetry.addData("Detected", true);
            telemetry.addData("ID", target.id);
            telemetry.addData("X", target.x);
            telemetry.addData("Y", target.y);
        } else {
            packet.put("Detected", false);
            telemetry.addLine("No object detected");
        }

        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }
}
