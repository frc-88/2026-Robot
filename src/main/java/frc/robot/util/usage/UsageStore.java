package frc.robot.util.usage;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Reader;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.nio.file.AtomicMoveNotSupportedException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.Properties;

/**
 * Saves usage counters to files on the roboRIO so they survive reboots and code deploys.
 *
 * <p>Plain Java only (no WPILib), so it can be tested on a laptop. Everything lives in one folder
 * (on the robot, {@code /home/lvuser/usage}):
 *
 * <ul>
 *   <li>{@value #STATE_FILE} — the current counters, as a text "properties" file (one {@code
 *       key=value} per line). Readable and hand-editable over SFTP if it ever needs fixing.
 *   <li>{@value #BACKUP_FILE} — the previous version of the state file.
 *   <li>{@value #SERVICE_LOG_FILE} — one row per inspection or replacement, never rewritten. This
 *       is the dataset for learning how far parts last. It opens directly in a spreadsheet.
 * </ul>
 *
 * <p><b>Power-loss safety.</b> The state file is never edited in place. A save writes a new
 * temporary file, forces it onto the flash, copies the current file to the backup, then renames the
 * temporary file over the current one. A rename is all-or-nothing, so a power cut at any point
 * leaves either the old or the new file intact, never a half-written one.
 *
 * <p>Methods are {@code synchronized} because the tracker saves on a background thread.
 */
public final class UsageStore {
  public static final String STATE_FILE = "usage.properties";
  public static final String BACKUP_FILE = "usage.properties.bak";
  public static final String SERVICE_LOG_FILE = "service-log.csv";

  /** Written into every state file so a future format change can be recognized. */
  public static final String SCHEMA_KEY = "schema";

  public static final String SCHEMA_VERSION = "1";

  /**
   * Columns of the service log. {@code since_replacement_m} and {@code lifetime_m} are the
   * distances at the moment of the action (for a replacement: the distance the old part reached).
   * {@code next_inspection_at_m} is the due point after the action.
   */
  public static final String SERVICE_LOG_HEADER =
      "timestamp,robot_serial,event,match,module,action,grade,"
          + "since_replacement_m,lifetime_m,next_inspection_at_m,old_label,new_label";

  /** Where a {@link #load()} found its data. */
  public enum Source {
    /** The normal state file. */
    PRIMARY,
    /** The state file was missing or unreadable; the backup copy was used. */
    BACKUP,
    /** Neither file was usable. The caller starts from zero. */
    NONE
  }

  /** Result of {@link #load()}: the saved values (empty if none) and where they came from. */
  public record Loaded(Properties properties, Source source) {}

  private final Path dir;

  /**
   * @param dir folder holding the files; created on the first save if it does not exist
   */
  public UsageStore(Path dir) {
    this.dir = dir;
  }

  /** Reads the state file, falling back to the backup if the state file is missing or damaged. */
  public synchronized Loaded load() {
    Properties primary = tryRead(dir.resolve(STATE_FILE));
    if (primary != null) {
      return new Loaded(primary, Source.PRIMARY);
    }
    Properties backup = tryRead(dir.resolve(BACKUP_FILE));
    if (backup != null) {
      return new Loaded(backup, Source.BACKUP);
    }
    return new Loaded(new Properties(), Source.NONE);
  }

  /** Returns the file's contents, or null if it is missing, unreadable, or not our format. */
  private static Properties tryRead(Path file) {
    if (!Files.isRegularFile(file)) {
      return null;
    }
    Properties props = new Properties();
    try (Reader reader = Files.newBufferedReader(file, StandardCharsets.UTF_8)) {
      props.load(reader);
    } catch (IOException | IllegalArgumentException e) {
      return null;
    }
    return SCHEMA_VERSION.equals(props.getProperty(SCHEMA_KEY)) ? props : null;
  }

  /** Saves {@code state} as the new state file (see the class comment for how this stays safe). */
  public synchronized void save(Properties state) throws IOException {
    Files.createDirectories(dir);
    Properties toWrite = new Properties();
    toWrite.putAll(state);
    toWrite.setProperty(SCHEMA_KEY, SCHEMA_VERSION);

    Path primary = dir.resolve(STATE_FILE);
    Path temp = dir.resolve(STATE_FILE + ".tmp");
    try (FileOutputStream out = new FileOutputStream(temp.toFile());
        Writer writer = new OutputStreamWriter(out, StandardCharsets.UTF_8)) {
      toWrite.store(writer, "Component usage counters (TreadUsageTracker). Units: meters.");
      writer.flush();
      out.getFD().sync(); // make sure it is on the flash before it replaces the old file
    }
    if (Files.isRegularFile(primary)) {
      Files.copy(primary, dir.resolve(BACKUP_FILE), StandardCopyOption.REPLACE_EXISTING);
    }
    try {
      Files.move(temp, primary, StandardCopyOption.ATOMIC_MOVE);
    } catch (AtomicMoveNotSupportedException e) {
      Files.move(temp, primary, StandardCopyOption.REPLACE_EXISTING);
    }
  }

  /** Adds one row to the service log, writing the header first if the log is new. */
  public synchronized void appendServiceRecord(String csvRow) throws IOException {
    Files.createDirectories(dir);
    Path log = dir.resolve(SERVICE_LOG_FILE);
    boolean isNew = !Files.exists(log);
    try (FileOutputStream out = new FileOutputStream(log.toFile(), true);
        Writer writer = new OutputStreamWriter(out, StandardCharsets.UTF_8)) {
      if (isNew) {
        writer.write(SERVICE_LOG_HEADER + "\n");
      }
      writer.write(csvRow + "\n");
      writer.flush();
      out.getFD().sync();
    }
  }

  /** Joins fields into one CSV row, quoting any field that contains a comma, quote, or newline. */
  public static String csvRow(String... fields) {
    StringBuilder row = new StringBuilder();
    for (int i = 0; i < fields.length; i++) {
      if (i > 0) {
        row.append(',');
      }
      String field = fields[i] == null ? "" : fields[i];
      if (field.contains(",")
          || field.contains("\"")
          || field.contains("\n")
          || field.contains("\r")) {
        row.append('"').append(field.replace("\"", "\"\"")).append('"');
      } else {
        row.append(field);
      }
    }
    return row.toString();
  }
}
