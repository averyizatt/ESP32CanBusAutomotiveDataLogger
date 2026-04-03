#pragma once

// Call once when entering CAN hack mode
void canCommanderSetup();

// Call continuously while in CAN hack mode
void canCommanderLoop();
