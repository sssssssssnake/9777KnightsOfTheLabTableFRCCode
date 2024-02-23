package frc.robot;

public class HardwareMap {

    //Neo Brushless Motor
    public static int mDeliveryRight = 0;
    public static int mDeliveryLeft = 0;

    //Neo Brushless Motor
    public static int mIntake = 0;

    //Neo Brushless Motor
    public static int mHangRight = 0;
    public static int mHangLeft = 0;


    public DeliverySubsystem delivery;
    public IntakeSubsystem intake;
    public HangSubsystem hang;

    HardwareMap(){
        delivery = new DeliverySubsystem(mDeliveryRight, mDeliveryLeft);
        intake = new IntakeSubsystem(mIntake);
        hang = new HangSubsystem(mHangRight, mHangLeft);

    }

}
