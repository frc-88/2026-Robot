package frc.robot.util.usage;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Properties;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

class UsageStoreTest {
  @TempDir Path dir;

  private static Properties state(String value) {
    Properties props = new Properties();
    props.setProperty("Tread/FL.lifetime", value);
    return props;
  }

  @Test
  void emptyFolderLoadsNothing() {
    UsageStore.Loaded loaded = new UsageStore(dir).load();
    assertEquals(UsageStore.Source.NONE, loaded.source());
    assertTrue(loaded.properties().isEmpty());
  }

  @Test
  void missingFolderLoadsNothingAndIsCreatedOnSave() throws IOException {
    Path nested = dir.resolve("usage");
    UsageStore store = new UsageStore(nested);
    assertEquals(UsageStore.Source.NONE, store.load().source());
    store.save(state("1.0"));
    assertTrue(Files.isRegularFile(nested.resolve(UsageStore.STATE_FILE)));
  }

  @Test
  void saveThenLoad() throws IOException {
    UsageStore store = new UsageStore(dir);
    store.save(state("123.5"));
    UsageStore.Loaded loaded = store.load();
    assertEquals(UsageStore.Source.PRIMARY, loaded.source());
    assertEquals("123.5", loaded.properties().getProperty("Tread/FL.lifetime"));
    assertEquals(UsageStore.SCHEMA_VERSION, loaded.properties().getProperty(UsageStore.SCHEMA_KEY));
    assertFalse(Files.exists(dir.resolve(UsageStore.STATE_FILE + ".tmp")));
  }

  @Test
  void secondSaveKeepsThePreviousVersionAsBackup() throws IOException {
    UsageStore store = new UsageStore(dir);
    store.save(state("1.0"));
    assertFalse(Files.exists(dir.resolve(UsageStore.BACKUP_FILE)));
    store.save(state("2.0"));
    assertEquals("2.0", store.load().properties().getProperty("Tread/FL.lifetime"));
    Properties backup = new Properties();
    try (var reader = Files.newBufferedReader(dir.resolve(UsageStore.BACKUP_FILE))) {
      backup.load(reader);
    }
    assertEquals("1.0", backup.getProperty("Tread/FL.lifetime"));
  }

  @Test
  void damagedStateFileFallsBackToBackup() throws IOException {
    UsageStore store = new UsageStore(dir);
    store.save(state("1.0"));
    store.save(state("2.0"));
    Files.writeString(dir.resolve(UsageStore.STATE_FILE), "not our file", StandardCharsets.UTF_8);
    UsageStore.Loaded loaded = store.load();
    assertEquals(UsageStore.Source.BACKUP, loaded.source());
    assertEquals("1.0", loaded.properties().getProperty("Tread/FL.lifetime"));
  }

  @Test
  void deletedStateFileFallsBackToBackup() throws IOException {
    UsageStore store = new UsageStore(dir);
    store.save(state("1.0"));
    store.save(state("2.0"));
    Files.delete(dir.resolve(UsageStore.STATE_FILE));
    assertEquals(UsageStore.Source.BACKUP, store.load().source());
  }

  @Test
  void serviceLogWritesHeaderOnceThenAppends() throws IOException {
    UsageStore store = new UsageStore(dir);
    store.appendServiceRecord(UsageStore.csvRow("a", "b"));
    store.appendServiceRecord(UsageStore.csvRow("c", "d"));
    List<String> lines =
        Files.readAllLines(dir.resolve(UsageStore.SERVICE_LOG_FILE), StandardCharsets.UTF_8);
    assertEquals(List.of(UsageStore.SERVICE_LOG_HEADER, "a,b", "c,d"), lines);
  }

  @Test
  void csvRowQuotesOnlyWhenNeeded() {
    assertEquals("plain,,x", UsageStore.csvRow("plain", null, "x"));
    assertEquals("\"has,comma\",\"say \"\"hi\"\"\"", UsageStore.csvRow("has,comma", "say \"hi\""));
  }

  @Test
  void headerHasTwelveColumns() {
    assertEquals(12, UsageStore.SERVICE_LOG_HEADER.split(",").length);
  }
}
