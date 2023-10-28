// This class was written by Google Bard with a lot of prompting by me.
// I renamed variables after the fact and fixed some obvious mistakes
// Not bad though

#include "KeyValueReader.h"

KeyValueReader::KeyValueReader(const char *filename)
{
  file = SD.open(filename, FILE_READ);
  if (!file)
  {
    Serial.println("Failed to open file");
  }
}

// A custom fgets() function for ESP32.
char *KeyValueReader::fgets(char *buffer, int size)
{
  int index = 0;
  char character;

  while (file.available() && index < size - 1)
  {
    character = file.read();
    if (character == '\n')
    {
      buffer[index] = '\0';
      return buffer;
    }

    buffer[index] = character;
    index++;
  }

  // If we reach the end of the line without finding a newline character, null-terminate the string and return.
  buffer[index] = '\0';
  return buffer;
}

// Skips empty lines and lines starting with a hash.
void KeyValueReader::skipEmptyAndCommentLines()
{
  char character;

  while (file.available())
  {
    character = file.peek();

    if (character == '\n' || character == '#')
    {
      fgets(line, sizeof(line));
    }
    else
    {
      break;
    }
  }
}

void KeyValueReader::process(void (*callback)(const char *, const char *))
{
  skipEmptyAndCommentLines();

  while (file.available())
  {
    fgets(line, sizeof(line));

    // Split the line into key and value parts.
    char *key = strtok(line, "=");
    char *value = strtok(NULL, "=");

    // Call the callback function with the key and value parts.
    callback(key, value);

    skipEmptyAndCommentLines();
  }
}

void KeyValueReader::close()
{
  file.close();
}