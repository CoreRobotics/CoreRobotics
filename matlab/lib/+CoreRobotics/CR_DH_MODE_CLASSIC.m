function v = CR_DH_MODE_CLASSIC()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 32);
  end
  v = vInitialized;
end
