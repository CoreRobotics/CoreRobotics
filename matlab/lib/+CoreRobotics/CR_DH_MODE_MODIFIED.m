function v = CR_DH_MODE_MODIFIED()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 34);
  end
  v = vInitialized;
end
