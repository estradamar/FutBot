package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.FutBotVision;

/**
 * OpMode autónomo: anotar en la portería AMARILLA, defender la azul.
 */
@Autonomous(name = "FutBot - Portería AMARILLA", group = "FutBot")
public class FutBotAutoAmarillo extends FutBotAuto {

    @Override
    protected int getIdPorteriaObjetivo() { return FutBotVision.ID_PORTERIA_AMARILLA; }

    @Override
    protected int getIdPorteriaPropia()   { return FutBotVision.ID_PORTERIA_AZUL; }
}
