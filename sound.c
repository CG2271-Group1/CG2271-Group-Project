#include "sound.h"




uint8_t megDurations[] = {
    1, 1, 1, 3, 1, 3, 1, 2, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1, 2, 1, 1
};

uint32_t megMelody[] = {
    N_C4, N_E4, N_G4, N_A4, N_G4, REST, N_C4, N_D4, N_C4, N_E4, REST, N_E4, N_G4, N_A4, N_B4, N_A4, N_G4, N_E4, N_D4, N_C4, N_D4, N_E4, N_C4
};

uint32_t megNumNotes = sizeof(megMelody) / sizeof(megMelody[0]);



uint32_t windowsMelody[] = {
  REST, N_GS5, N_DS5, N_GS4, N_AS4
};

uint8_t windowsDurations[] = {
 9, 9, 9, 9, 6
};

uint8_t windowsNumNotes = 5;


uint32_t coffinMelody[] = {
  N_AS4, N_AS4, N_AS4, N_AS4,
  N_AS4, N_AS4, N_AS4, N_AS4,
  N_D5, N_D5, N_D5, N_D5,
  N_C5, N_C5, N_C5, N_C5, 
  N_F5, N_F5, N_F5, N_F5, 
  N_G5, N_G5, N_G5, N_G5,
  N_G5, N_G5, N_G5, N_G5, 
  N_G5, N_G5, N_G5, N_G5, 
  N_C5, N_AS4, N_A4, N_F4,
  N_G4, REST, N_G4, N_D5,
  N_C5, REST, N_AS4, REST,
  N_A4, REST, N_A4, N_A4,
  N_C5, REST, N_AS4, N_A4, 
  N_G4, REST, N_G4, N_AS5,
  N_A5, N_AS5, N_A5, N_AS5,
  N_G4, REST, N_G4, N_AS5,
  N_A5, N_AS5, N_A5, N_AS5,
  N_G4, REST, N_G4, N_D5,
  N_C5, REST, N_AS4, REST,
  N_A4, REST, N_A4, N_A4,
  N_C5, REST, N_AS4, N_A4, 
  N_G4, REST, N_G4, N_AS5,
  N_A5, N_AS5, N_A5, N_AS5,
  N_G4, REST, N_G4, N_AS5,
  N_A5, N_AS5, N_A5, N_AS5
 };

uint8_t coffinDurations[] = {
  10,10,10,10,
  10,10,10,10,
  10,10,10,10,
  10,10,10,10,
  10,10,10,10,
  10,10,10,10,
  10,10,10,10,
  10,10,10,10,
  10,10,10,10,
  10,10,10,10,
  10,10,10,10,
  10,10,10,10,
  10,10,10,10,
  10,10,10,10,
  10,10,10,10,
  10,10,10,10,
  10,10,10,10,
  10,10,10,10,
  10,10,10,10,
  10,10,10,10,
  10,10,10,10,
  10,10,10,10,
  10,10,10,10,
  10,10,10,10,
  10,10,10,10,
  };

uint8_t coffinNumNotes = 100;

void initPWMSound() {
  uint8_t pins[] = {12}, channels[] = {0};
  
  initPWMAll(PORTA, 3, pins, 1, TPM1, channels, 1);
}

void setPWMFreq(TPM_Type *timer, uint8_t channel, uint32_t freq) {
  TPM_CnV_REG(timer,channel) = FREQ_2_MOD(freq)/4; // 25% duty cycle
  timer->MOD = FREQ_2_MOD(freq);
}


void playTone(TPM_Type *timer, uint8_t channel, uint32_t tone, uint8_t duration) {
  setPWMFreq(timer, channel, tone);
  osDelay(150);
}

void playToneNoSem(TPM_Type *timer, uint8_t channel, uint32_t tone, uint8_t duration) {
  setPWMFreq(timer, channel, tone);
  osDelay(250);
}

void playToneDelay2(TPM_Type *timer, uint8_t channel, uint32_t tone, uint8_t duration) {
  setPWMFreq(timer, channel, tone);
  delay2(duration*0xFFF);
}

void playSong(TPM_Type *timer, uint8_t channel, uint32_t numNotes, 
    uint8_t noteDurations[], uint32_t melody[]) {
  for (int i = 0; i < numNotes; i++) {
    osSemaphoreAcquire(musicSem, osWaitForever);
    uint8_t noteDuration = TEMPO / noteDurations[i];
    playTone(timer, channel, melody[i], noteDuration);
    osSemaphoreRelease(musicSem);
    osDelay(noteDuration * 1.45);
  }
}
    
void playSongDelay2(TPM_Type *timer, uint8_t channel, uint32_t numNotes, 
    uint8_t noteDurations[], uint32_t melody[]) {
  for (int i = 0; i < numNotes; i++) {
    uint8_t noteDuration = TEMPO / noteDurations[i];
    playToneDelay2(timer, channel, melody[i], noteDuration);
    delay2(noteDuration * 1.45);
  }
}

void playSongNoSem(TPM_Type *timer, uint8_t channel, uint32_t numNotes, 
    uint8_t noteDurations[], uint32_t melody[]) {
  for (int i = 0; i < numNotes; i++) {
    uint8_t noteDuration = TEMPO / noteDurations[i];
    playToneNoSem(timer, channel, melody[i], noteDuration);
    osDelay(noteDuration * 1.45);
  }
}

// Running song
void playMegalovania() {
  playSong(TPM1, 0, megNumNotes, megDurations, megMelody);
}

// Connection song
void playWindowsDelay2() {
  playSongDelay2(TPM1, 0, windowsNumNotes, windowsDurations, windowsMelody);
}

// For debugging/testing only
void playWindows() {
  playSong(TPM1, 0, windowsNumNotes, windowsDurations, windowsMelody);
}

// Ending song
void playCoffin() {
  osSemaphoreAcquire(musicSem, osWaitForever);
  playSongNoSem(TPM1, 0, coffinNumNotes, coffinDurations, coffinMelody);
  osSemaphoreRelease(musicSem);
}

void offSound() {
  setDuty(TPM1, 0, 0);
}
