package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "Hive Up", preselectTeleOp = "TeleOp_fieldCentric")
public class HiveUpAutonomous extends OpMode {
    interface Route {
        void init(HardwareMap hardwareMap, Telemetry telemetry, AutonParking parking);
        void initLoop(OpModeStorage.Alliance allianceOverride);
        void start();
        void loop();
        void stop(boolean saveEndPose);
    }

    private enum Mode {
        ONE_SIDE("Shoot 1 side (Hive Up)"),
        TWO_SIDES("Shoot 2 sides (Hive Up Two Tips)");

        final String label;

        Mode(String label) {
            this.label = label;
        }
    }

    private AutonInitMenu menu;
    private AutonParking parking;
    private Mode mode;
    private Route route;
    private boolean started;

    @Override
    public void init() {
        started = false;
        menu = new AutonInitMenu(true);
        parking = new AutonParking();
        select(Mode.ONE_SIDE);
    }

    @Override
    public void init_loop() {
        menu.update(gamepad2);
        parking.selectParkOne(menu.parkOne());
        select(menu.shootTwoSides() ? Mode.TWO_SIDES : Mode.ONE_SIDE);
        menu.show(telemetry);
        route.initLoop(menu.alliance());
    }

    @Override
    public void start() {
        route.start();
        started = true;
    }

    @Override
    public void loop() {
        telemetry.addData("Shooting mode", mode.label);
        route.loop();
    }

    @Override
    public void stop() {
        if (route != null) {
            route.stop(started);
            route = null;
        }
    }

    private void select(Mode selected) {
        if (route != null && mode == selected) {
            return;
        }
        if (route != null) {
            route.stop(false);
        }
        mode = selected;
        route = selected == Mode.ONE_SIDE ? new hiveup() : new hiveup_two_tips();
        menu.show(telemetry);
        route.init(hardwareMap, telemetry, parking);
    }

}
