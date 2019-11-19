package org.firstinspires.ftc.teamcode.ftc2753.roadrunner.drive;

import com.qualcomm.robotcore.hardware.configuration.DistributorInfo;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFPositionParams;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFVelocityParams;
import com.qualcomm.robotcore.hardware.configuration.annotations.MotorType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

/**
 * Created by David Zheng | FTC 2753 Team Overdrive on 11/16/2019.
 */

@MotorType(ticksPerRev=2786, gearing=19.2, maxRPM=312, orientation= Rotation.CCW)

@DeviceProperties(xmlTag="goBILDA520200020019Motor", name="GoBILDA 5202 19.2:1 Motor",
        description = "GoBILDA 5202-0002-0019 Yellow Jacket Planetary Gear Motor (19.2:1 Ratio, 312 RPM, 3.3 - 5V Encoder)")

@DistributorInfo(distributor="goBILDA_distributor", model="goBILDA-5202-0002-0019",
        url="https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-312-rpm-3-3-5v-encoder/")

@ExpansionHubPIDFVelocityParams(P=2.0, I=0.5, F=11.1)
@ExpansionHubPIDFPositionParams(P=5.0)

public interface GoBILDA520200020019 {}
