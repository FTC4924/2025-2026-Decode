package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.EncoderGroup;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxQuadratureEncoderGroup;
import com.acmerobotics.roadrunner.ftc.OTOSEncoderGroup;
import com.acmerobotics.roadrunner.ftc.OTOSIMU;
import com.acmerobotics.roadrunner.ftc.PinpointEncoderGroup;
import com.acmerobotics.roadrunner.ftc.PinpointIMU;
import com.acmerobotics.roadrunner.ftc.PinpointView;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.roadrunner.IHDrive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public final class TuningOpModes {
    // TODO: change this to TankDrive.class if you're using tank
    public static final Class<?> DRIVE_CLASS = IHDrive.class;

    public static final String GROUP = "quickstart";
    public static final boolean DISABLED = false;

    private TuningOpModes() {}

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    private static PinpointView makePinpointView(PinpointLocalizer pl) {
        return new PinpointView() {
            GoBildaPinpointDriver.EncoderDirection parDirection = pl.initialParDirection;
            GoBildaPinpointDriver.EncoderDirection perpDirection = pl.initialPerpDirection;

            @Override
            public void update() {
                pl.driver.update();
            }

            @Override
            public int getParEncoderPosition() {
                return pl.driver.getEncoderX();
            }

            @Override
            public int getPerpEncoderPosition() {
                return pl.driver.getEncoderY();
            }

            @Override
            public float getHeadingVelocity(UnnormalizedAngleUnit unit) {
                return (float) pl.driver.getHeadingVelocity(unit);
            }

            @Override
            public void setParDirection(@NonNull DcMotorSimple.Direction direction) {
                parDirection = direction == DcMotorSimple.Direction.FORWARD ?
                        GoBildaPinpointDriver.EncoderDirection.FORWARD :
                        GoBildaPinpointDriver.EncoderDirection.REVERSED;
                pl.driver.setEncoderDirections(parDirection, perpDirection);
            }

            @Override
            public DcMotorSimple.Direction getParDirection() {
                return parDirection == GoBildaPinpointDriver.EncoderDirection.FORWARD ?
                        DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
            }

            @Override
            public void setPerpDirection(@NonNull DcMotorSimple.Direction direction) {
                perpDirection = direction == DcMotorSimple.Direction.FORWARD ?
                        GoBildaPinpointDriver.EncoderDirection.FORWARD :
                        GoBildaPinpointDriver.EncoderDirection.REVERSED;
                pl.driver.setEncoderDirections(parDirection, perpDirection);
            }

            @Override
            public DcMotorSimple.Direction getPerpDirection() {
                return perpDirection == GoBildaPinpointDriver.EncoderDirection.FORWARD ?
                        DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
            }
        };
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (DISABLED) return;

        DriveViewFactory dvf;
        if (DRIVE_CLASS.equals(IHDrive.class)) {
            dvf = hardwareMap -> {
                IHDrive drive = new IHDrive(hardwareMap, new Pose2d(0, 0, 0));
                LazyImu lazyImu = drive.lazyImu;

                List<EncoderGroup> encoderGroups = new ArrayList<>();
                List<EncoderRef> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
                List<EncoderRef> parEncs = new ArrayList<>(), perpEncs = new ArrayList<>();
                if (drive.localizer instanceof PinpointLocalizer) {
                    PinpointView pv = makePinpointView((PinpointLocalizer) drive.localizer);
                    encoderGroups.add(new PinpointEncoderGroup(pv));
                    parEncs.add(new EncoderRef(0, 0));
                    perpEncs.add(new EncoderRef(0, 1));
                    lazyImu = new PinpointIMU(pv);
                } else {
                    throw new RuntimeException("unknown localizer: " + drive.localizer.getClass().getName());
                }

                return new DriveView(
                        DriveType.IH,
                        IHDrive.PARAMS.inPerTick,
                        IHDrive.PARAMS.maxWheelVel,
                        IHDrive.PARAMS.minProfileAccel,
                        IHDrive.PARAMS.maxProfileAccel,
                        encoderGroups,
                        Collections.singletonList(drive.left),
                        Collections.singletonList(drive.right),
                        Collections.singletonList(drive.middle),
                        leftEncs,
                        rightEncs,
                        parEncs,
                        perpEncs,
                        lazyImu,
                        drive.voltageSensor,
                        () -> new MotorFeedforward(IHDrive.PARAMS.kS,
                                IHDrive.PARAMS.kV / IHDrive.PARAMS.inPerTick,
                                IHDrive.PARAMS.kA / IHDrive.PARAMS.inPerTick),
                        0
                );
            };
        } else {
            throw new RuntimeException();
        }

        manager.register(metaForClass(AngularRampLogger.class), new AngularRampLogger(dvf));
        manager.register(metaForClass(ForwardPushTest.class), new ForwardPushTest(dvf));
        manager.register(metaForClass(ForwardRampLogger.class), new ForwardRampLogger(dvf));
        manager.register(metaForClass(LateralPushTest.class), new LateralPushTest(dvf));
        manager.register(metaForClass(LateralRampLogger.class), new LateralRampLogger(dvf));
        manager.register(metaForClass(ManualFeedforwardTuner.class), new ManualFeedforwardTuner(dvf));
        manager.register(metaForClass(MecanumMotorDirectionDebugger.class), new MecanumMotorDirectionDebugger(dvf));
        manager.register(metaForClass(DeadWheelDirectionDebugger.class), new DeadWheelDirectionDebugger(dvf));

        manager.register(metaForClass(ManualFeedbackTuner.class), ManualFeedbackTuner.class);
        manager.register(metaForClass(SplineTest.class), SplineTest.class);
        manager.register(metaForClass(LocalizationTest.class), LocalizationTest.class);

        manager.register(metaForClass(OTOSAngularScalarTuner.class), new OTOSAngularScalarTuner(dvf));
        manager.register(metaForClass(OTOSLinearScalarTuner.class), new OTOSLinearScalarTuner(dvf));
        manager.register(metaForClass(OTOSHeadingOffsetTuner.class), new OTOSHeadingOffsetTuner(dvf));
        manager.register(metaForClass(OTOSPositionOffsetTuner.class), new OTOSPositionOffsetTuner(dvf));

        FtcDashboard.getInstance().withConfigRoot(configRoot -> {
            for (Class<?> c : Arrays.asList(
                    AngularRampLogger.class,
                    ForwardRampLogger.class,
                    LateralRampLogger.class,
                    ManualFeedforwardTuner.class,
                    MecanumMotorDirectionDebugger.class,
                    ManualFeedbackTuner.class
            )) {
                configRoot.putVariable(c.getSimpleName(), ReflectionConfig.createVariableFromClass(c));
            }
        });
    }
}
