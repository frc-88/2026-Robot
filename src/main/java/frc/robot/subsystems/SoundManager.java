package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayDeque;
import java.util.Queue;
import java.util.function.BooleanSupplier;

public class SoundManager extends SubsystemBase {
  public static class Sounds {
    public static String BASS = "bass";
    public static String ERROR = "BigError";
  }

  boolean goodToGo = false;
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("Sounds");
  private StringTopic topic = table.getStringTopic("Sounds");
  private StringSubscriber sub = topic.subscribe("");
  private StringPublisher pub = topic.publish();

  private static Queue<String> toPlay = new ArrayDeque<>();

  public SoundManager() {}

  public static void playSoundIf(String sound, BooleanSupplier condition) {
    new Trigger(condition).onTrue(new InstantCommand(() -> toPlay.add(sound)));
  }

  public static void playSound(String sound) {
    toPlay.add(sound);
  }

  @Override
  public void periodic() {
    String current = sub.get();
    if (current.equals("None")) {
      goodToGo = true;
    } else {
      goodToGo = false;
    }

    if (goodToGo) {
      pub.set(toPlay.poll());
    }
  }
}
