function v = CR_DH_MODE_MODIFIED()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 32);
  end
  v = vInitialized;
end
