  switch (ID) {

    // throttle pedal
    case 0x40:
      Eng_Spd = uint16_t(buf[D] << 8 | buf[C]) & 0x3FFF;
      Accel_Pos = buf[E] / 2.55;
      F_Accel = (buf[H] & 0xC0) != 0xC0;
      break;

    // steering
    case 0x138:
      Steer_Ang = int16_t(buf[D] << 8 | buf[C]) * -0.1;
      Yaw_Rate = int16_t(buf[F] << 8 | buf[E]) * -0.2725;
      break;

    // brakes
    case 0x139:
      Veh_Spd = (uint16_t(buf[D] << 8 | buf[C]) & 0x1FFF) * 0.05625; // kph;
      F_Brake = (buf[E] & 0x4) == 4;
      Brake_Pos = min(buf[F] / 0.7, 100);
      break;

    // accelerometers
    case 0x13B:
      Lat_Acc = (int8_t(buf[G]) * -0.1) / g;
      Lng_Acc = (int8_t(buf[H]) * -0.1) / g;
      break;

    // transmission
    case 0x241:
      Gear = buf[E] >> 3 & 0x7;
      F_Clutch = ((buf[F] & 0x80) / 1.28) == 100;
      break;

    // brightness
    case 0x390:
      Dash_Bright = buf[F];
      F_Light = buf[G] & 0x10;
      break;

    // doors
    case 0x3AC:
      F_DrivDoor = buf[E] & 0x1;
      F_PassDoor = buf[E] & 0x2;
      F_Trunk = buf[E] & 0x20;

      F_Reverse = buf[G] & 0x10;
      F_Park = buf[G] & 0x20;
      F_Brake = buf[G] & 0x40;

      F_DRL = buf[H] & 0x1;
      F_Headlights = buf[H] & 0x2;
      F_Highbeams = buf[H] & 0x4;
      F_Wipers = buf[H] & 0x40;

      break;
  }