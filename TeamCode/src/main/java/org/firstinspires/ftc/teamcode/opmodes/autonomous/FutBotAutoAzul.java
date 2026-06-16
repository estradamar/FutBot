package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.FutBotVision;

/**
 * OpMode autónomo: anotar en la portería AZUL, defender la amarilla.
 */
@Autonomous(name = "FutBot - Portería AZUL", group = "FutBot")
public class FutBotAutoAzul extends FutBotAuto {

    @Override
    protected int getIdPorteriaObjetivo() { return FutBotVision.ID_PORTERIA_AZUL; }

    @Override
    protected int getIdPorteriaPropia()   { return FutBotVision.ID_PORTERIA_AMARILLA; }
}
