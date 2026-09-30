package frc.robot.util.usage;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Properties;
import org.junit.jupiter.api.Test;

class UsageMeterTest {
  private static final double EPS = 1e-9;
  private static final UsageMeter.Schedule SCHEDULE =
      new UsageMeter.Schedule(2000.0, 2000.0, 1000.0, 250.0);

  private static UsageMeter newMeter() {
    return new UsageMeter("Tread/FL", SCHEDULE);
  }

  @Test
  void newMeterStartsAtZeroWithFirstInspectionScheduled() {
    UsageMeter meter = newMeter();
    assertEquals(0.0, meter.getLifetime(), EPS);
    assertEquals(0.0, meter.getSinceReplacement(), EPS);
    assertEquals(2000.0, meter.getNextInspectionAt(), EPS);
    assertEquals(UsageMeter.UNKNOWN_LABEL, meter.getLabel());
    assertNull(meter.getLastGrade());
    assertFalse(meter.isInspectionDue());
  }

  @Test
  void addAccumulatesAndIgnoresBadValues() {
    UsageMeter meter = newMeter();
    meter.add(1.5);
    meter.add(2.5);
    meter.add(-3.0);
    meter.add(0.0);
    meter.add(Double.NaN);
    meter.add(Double.POSITIVE_INFINITY);
    assertEquals(4.0, meter.getLifetime(), EPS);
    assertEquals(4.0, meter.getSinceReplacement(), EPS);
  }

  @Test
  void inspectionBecomesDueAtTheThreshold() {
    UsageMeter meter = newMeter();
    meter.add(1999.0);
    assertFalse(meter.isInspectionDue());
    assertEquals(1.0, meter.remainingUntilInspection(), EPS);
    meter.add(1.0);
    assertTrue(meter.isInspectionDue());
  }

  @Test
  void snoozeCountsFromTheInspectionPointNotTheOldThreshold() {
    // Warning fires at 2.0 km, but the team only gets to it at 2.6 km and grades it Good.
    UsageMeter meter = newMeter();
    meter.add(2600.0);
    assertTrue(meter.isInspectionDue());
    meter.inspected(Grade.GOOD);
    assertEquals(4600.0, meter.getNextInspectionAt(), EPS); // 2.6 + 2.0, not 2.0 + 2.0
    assertFalse(meter.isInspectionDue());
    assertEquals(Grade.GOOD, meter.getLastGrade());
    assertEquals(2600.0, meter.getLastGradedAt(), EPS);
    assertEquals(2600.0, meter.getSinceReplacement(), EPS); // inspecting does not reset the count
  }

  @Test
  void eachGradeUsesItsOwnSnooze() {
    UsageMeter meter = newMeter();
    meter.add(2000.0);
    meter.inspected(Grade.WORN);
    assertEquals(3000.0, meter.getNextInspectionAt(), EPS);
    meter.add(1000.0);
    meter.inspected(Grade.POOR);
    assertEquals(3250.0, meter.getNextInspectionAt(), EPS);
    meter.add(250.0);
    assertTrue(meter.isInspectionDue());
  }

  @Test
  void replacementResetsSinceReplacementButKeepsLifetime() {
    UsageMeter meter = newMeter();
    meter.add(3000.0);
    meter.inspected(Grade.POOR);
    meter.replaced("tread-B");
    assertEquals(3000.0, meter.getLifetime(), EPS);
    assertEquals(0.0, meter.getSinceReplacement(), EPS);
    assertEquals(2000.0, meter.getNextInspectionAt(), EPS);
    assertEquals("tread-B", meter.getLabel());
    assertNull(meter.getLastGrade());
    assertTrue(Double.isNaN(meter.getLastGradedAt()));
  }

  @Test
  void blankReplacementLabelBecomesUnknown() {
    UsageMeter meter = newMeter();
    meter.replaced("   ");
    assertEquals(UsageMeter.UNKNOWN_LABEL, meter.getLabel());
  }

  @Test
  void saveAndRestoreRoundTrip() {
    UsageMeter original = newMeter();
    original.add(1234.5);
    original.replaced("tread-A");
    original.add(987.25);
    original.inspected(Grade.WORN);

    Properties saved = new Properties();
    original.writeTo(saved);
    UsageMeter restored = newMeter();
    assertTrue(restored.readFrom(saved));

    assertEquals(original.getLifetime(), restored.getLifetime(), EPS);
    assertEquals(original.getSinceReplacement(), restored.getSinceReplacement(), EPS);
    assertEquals(original.getNextInspectionAt(), restored.getNextInspectionAt(), EPS);
    assertEquals("tread-A", restored.getLabel());
    assertEquals(Grade.WORN, restored.getLastGrade());
    assertEquals(987.25, restored.getLastGradedAt(), EPS);
  }

  @Test
  void roundTripWithNoInspectionYet() {
    UsageMeter original = newMeter();
    original.add(10.0);
    Properties saved = new Properties();
    original.writeTo(saved);
    UsageMeter restored = newMeter();
    assertTrue(restored.readFrom(saved));
    assertNull(restored.getLastGrade());
    assertTrue(Double.isNaN(restored.getLastGradedAt()));
  }

  @Test
  void restoreIgnoresOtherMeters() {
    UsageMeter frontLeft = newMeter();
    frontLeft.add(500.0);
    Properties saved = new Properties();
    frontLeft.writeTo(saved);
    UsageMeter frontRight = new UsageMeter("Tread/FR", SCHEDULE);
    assertFalse(frontRight.readFrom(saved)); // nothing saved under "Tread/FR."
    assertEquals(0.0, frontRight.getLifetime(), EPS);
  }

  @Test
  void invalidSavedValuesAreRejectedWithoutChangingTheMeter() {
    UsageMeter source = newMeter();
    source.add(100.0);
    Properties saved = new Properties();
    source.writeTo(saved);

    Properties negative = copy(saved);
    negative.setProperty("Tread/FL.lifetime", "-5");
    Properties garbage = copy(saved);
    garbage.setProperty("Tread/FL.sinceReplacement", "lots");
    Properties badGrade = copy(saved);
    badGrade.setProperty("Tread/FL.lastGrade", "EXCELLENT");
    Properties missing = copy(saved);
    missing.remove("Tread/FL.nextInspectionAt");
    Properties sinceAboveLifetime = copy(saved);
    sinceAboveLifetime.setProperty("Tread/FL.sinceReplacement", "200");

    for (Properties bad : new Properties[] {negative, garbage, badGrade, missing, sinceAboveLifetime}) {
      UsageMeter meter = newMeter();
      meter.add(7.0);
      assertFalse(meter.readFrom(bad));
      assertEquals(7.0, meter.getLifetime(), EPS);
      assertEquals(2000.0, meter.getNextInspectionAt(), EPS);
    }
  }

  @Test
  void gradeLabelsParse() {
    assertEquals(Grade.GOOD, Grade.fromLabel("Good"));
    assertEquals(Grade.WORN, Grade.fromLabel(" worn "));
    assertEquals(Grade.POOR, Grade.fromLabel("POOR"));
    assertNull(Grade.fromLabel(""));
    assertNull(Grade.fromLabel("(select grade)"));
    assertNull(Grade.fromLabel(null));
  }

  private static Properties copy(Properties source) {
    Properties result = new Properties();
    result.putAll(source);
    return result;
  }
}
