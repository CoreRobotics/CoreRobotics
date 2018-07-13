function v = CR_DH_FREE_ALPHA()
  persistent vInitialized;
  if isempty(vInitialized)
    vInitialized = CoreRoboticsMEX(0, 30);
  end
  v = vInitialized;
end
