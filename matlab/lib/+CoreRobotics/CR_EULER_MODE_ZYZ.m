function v = CR_EULER_MODE_ZYZ()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 12);
  end
  v = vInitialized;
end