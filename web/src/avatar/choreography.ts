export const sharedMotions = ['keonroot-challenge', 'dance', 'talk', 'pickup', 'walk', 'reach', 'cheer'] as const;
export type SharedMotion = typeof sharedMotions[number];

export const recordingMotion: Record<string, SharedMotion | 'greeting' | 'peace'> = {
  'keonroot-challenge': 'keonroot-challenge', greet: 'greeting', bye: 'greeting', attract: 'dance', thanks: 'cheer', marker_found: 'cheer',
  follow_start: 'walk', passing: 'walk', picking: 'pickup', holding: 'reach', released: 'reach',
  motion: 'dance', looking: 'talk', sos_sent: 'talk',
};

// Seconds in the recorded introduction. Leave the opening wave and closing bow intact.
export const introMoves: ReadonlyArray<[number, number, SharedMotion]> = [
  [9.4, 15.4, 'talk'], [17.7, 20.5, 'pickup'], [20.5, 23.2, 'reach'],
  [28.1, 30.1, 'dance'], [32.8, 34.8, 'walk'], [38.4, 40.9, 'cheer'],
];
