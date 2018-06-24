function v = CR_DH_FREE_ALPHA()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 28);
  end
  v = vInitialized;
end
