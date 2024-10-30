package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.List;

/**
 * Classe principal que representa o robô por completo, unindo
 * seus sistemas e configurações em um único objeto, que pode 
 * (e deve) ser usado em todo e qualquer tipo de OpMode que utilizamos
 */
public final class DestemidosBot {

    // lista com todos os hubs e seus IDs para fácil acesso
    private final List<LynxModule> allHubs;

    public final VoltageSensor voltageSensor;

    // Sistema do Drivetain
    public final Drivetrain drivetrain;

    // Sistema do braço
  //  public final ArmSystem armSystem;
    public final SimpleArm simpleArm;
    // Sistema de garra
    public final ServoSystem servoSystem;

    // Sistema de localização do IMU
    public final LocalizationSystem localizationSystem;
    public ArmSystem armSystem;


    /**
     * Construtor padrão que recebe um {@link HardwareMap}
     * e configura todos equipamentos e seus sistemas
     * @param hardwareMap presente em todo OpMode
     */
    public DestemidosBot(@NonNull HardwareMap hardwareMap){

        // listando todos os hubs conectados no robô
        allHubs = hardwareMap.getAll(LynxModule.class);

        //
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // definindo a cor das leds do hubs
        setHubColor(Color.CYAN);


        // inicializando os sistemas do robô
        drivetrain = new Drivetrain(hardwareMap);
        armSystem = new ArmSystem(hardwareMap);
        simpleArm = new SimpleArm(hardwareMap);
        servoSystem = new ServoSystem(hardwareMap);
        localizationSystem = new LocalizationSystem(hardwareMap, "imu");
    }

    /**
     * Ativa o modo de caching automático de ambos os Hubs
     */
    public void setBulkReadToAuto() {
        for (LynxModule robotHub : allHubs) {
            robotHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    /**
     * Ativa o modo de caching manual de ambos os Hubs
     * OBS: fica na resposabilidade do usuário, a limpeza desses dados em cache
     * se não tiver certeza de como realizar essa ação, recomendo o configurar no automático
     */
    public void setBulkCacheToManual() {
        for (LynxModule robotHub : allHubs) {
            robotHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    /**
     * Limpa o cache atual dos hubs
     */
    public void clearManualBulkCache() {
        for (LynxModule robotHub : allHubs) {
            robotHub.clearBulkCache();
        }
    }

    public void setHubColor(int color) {
        for (LynxModule hub: allHubs) {
            hub.setConstant(color);
        }
    }
}
