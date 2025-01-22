using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;

using Flat;
using Flat.Graphics;
using Flat.Input;
using System;
using System.Diagnostics;

using FlatPhysics;
using System.Collections.Generic;
using static FlatPhysics.FlatBody;
using FlatMath = FlatPhysics.FlatMath;
using System.Reflection.Metadata.Ecma335;
using System.Reflection.Metadata;
using System.Buffers.Binary;
using System.Threading;
using System.Diagnostics.Tracing;

namespace PhysicsTester
{
    public class Game1 : Game
    {
        private GraphicsDeviceManager graphics;
        private Screen screen;
        private Sprites sprites;
        private Shapes shapes;
        private Camera camera;
        private SpriteFont fontConsolas18;

        private FlatWorld world;
       
        private List<FlatEntity> entityList;
        private List<FlatEntity> entityRemovalList;
    
        private Stopwatch watch;

        private double totalWorldStepTime = 0d;
        private int totalBodyCount = 0;
        private int totalSampleCount = 0;
        private Stopwatch sampleTimer = new Stopwatch();

        private string worldStepTimeString = string.Empty;
        private string bodyCountString = string.Empty;

        public Game1()
        {
            graphics = new GraphicsDeviceManager(this);
            graphics.SynchronizeWithVerticalRetrace = true;
            Content.RootDirectory = "Content";
            IsMouseVisible = true;
            IsFixedTimeStep = true;

            const double UpdatesPerSecond = 60d;
            TargetElapsedTime = TimeSpan.FromTicks((long)Math.Round((double)TimeSpan.TicksPerSecond / UpdatesPerSecond));
   
        }

        protected override void Initialize()
        {

            //Console.WriteLine($"{x1}, {y1}");
            //    Console.WriteLine($"{a.X}, {a.Y}");
            // Console.WriteLine($"time: {watch.Elapsed.TotalMilliseconds}");
            //  Console.ReadKey(true);

            Window.Position = new Point(10, 40);

            FlatUtil.SetRelativeBackBufferSize(graphics, 0.85f);

            screen = new Screen(this, 1280, 768);
            sprites = new Sprites(this);
            shapes = new Shapes(this);
            camera = new Camera(screen);
            camera.Zoom = 20;

            camera.GetExtents(out float left, out float right, out float bottom, out float top);

            // TODO: Add your initialization logic here

           

            this.entityList = new List<FlatEntity>();
            entityRemovalList = new List<FlatEntity>();


            this.world = new FlatWorld();

            float padding = MathF.Abs(right - left) * 0.10f;

            #region groundbody

            if (!FlatBody.CreateBoxBody(right - left - padding * 2, 3f,
                1f, true, 0.5f, out FlatBody groundBody, out string errorMessage))
            { 
                throw new Exception(errorMessage);  
            }

            groundBody.MoveTo(new FlatVector(0, -10));

            this.world.AddBody(groundBody);
            this.entityList.Add(new FlatEntity(groundBody));


            #endregion

            #region ledgeBody

            if (!FlatBody.CreateBoxBody(20f, 2f,
                1f, true, 0.5f, out FlatBody ledgeBody1, out errorMessage))
            {
                throw new Exception(errorMessage);
            }
            ledgeBody1.MoveTo(new FlatVector(-10, 3));
            ledgeBody1.Rotate(-MathHelper.TwoPi / 20f);

            this.world.AddBody(ledgeBody1);
            this.entityList.Add(new FlatEntity(ledgeBody1));


            if (!FlatBody.CreateBoxBody(15f, 2f, 
                1f, true, 0.5f, out FlatBody ledgeBody2, out errorMessage))
            {
                throw new Exception(errorMessage);
            }
            ledgeBody2.MoveTo(new FlatVector(10, 10));
            ledgeBody2.Rotate(MathHelper.TwoPi / 20f);

            this.world.AddBody(ledgeBody2);
            this.entityList.Add(new FlatEntity(ledgeBody2));
           


            #endregion




            this.watch = new Stopwatch();
            sampleTimer.Start();
            base.Initialize();
        }

        protected override void LoadContent()
        {
            fontConsolas18 = Content.Load<SpriteFont>("consolas18");

            // TODO: use this.Content to load your game content here
        }

        protected override void Update(GameTime gameTime)
        {
            FlatKeyboard keyboard = FlatKeyboard.Instance;
            FlatMouse mouse = FlatMouse.Instance;

            keyboard.Update();
            mouse.Update();

            if (mouse.IsLeftMouseButtonPressed())
            {
                float width = RandomHelper.RandomSingle(2f,3f);
                float height = RandomHelper.RandomSingle(2f, 3f);

                FlatVector mouseWorldPosition = FlatConverter.ToFlatVector(mouse.GetMouseWorldPosition(this, this.screen, this.camera));


                this.entityList.Add(new FlatEntity(this.world,width,height,false,mouseWorldPosition));

            }

            if (mouse.IsRightMouseButtonPressed())
            {

                float radius = RandomHelper.RandomSingle(1.25f, 1.5f);
          

                FlatVector mouseWorldPosition = FlatConverter.ToFlatVector(mouse.GetMouseWorldPosition(this, this.screen, this.camera));

                this.entityList.Add(new FlatEntity(this.world, radius, false, mouseWorldPosition));

            }

            if (keyboard.IsKeyAvailable)
            {

                if (keyboard.IsKeyClicked(Keys.OemTilde))
                {
                    Console.WriteLine($"BodyCount : {this.world.BodyCount}");
                    Console.WriteLine($"StepTime : {Math.Round(this.watch.Elapsed.TotalMilliseconds,4)}");

                }


                if (keyboard.IsKeyClicked(Keys.Escape))
                {
                    Exit();
                }

                if (keyboard.IsKeyClicked(Keys.A))
                {
                    this.camera.IncZoom();
                }

                if (keyboard.IsKeyClicked(Keys.Z))
                {
                    this.camera.DecZoom();

                }

#if false

                float dx = 0f;
                float dy = 0f;
                float forceMagnitude = 1000f;

                if (keyboard.IsKeyDown(Keys.Left))
                {
                    dx--;
                }

                if (keyboard.IsKeyDown(Keys.Right))
                {
                    dx++;
                }

                if (keyboard.IsKeyDown(Keys.Up))
                {
                    dy++;
                }

                if (keyboard.IsKeyDown(Keys.Down))
                {
                    dy--;
                }

                if (!this.world.GetBody(0, out FlatBody body))
                {
                    throw new Exception("Could not find the body index");
                }



                if (dx != 0 || dy != 0)
                {
                    FlatVector forecdirection = FlatMath.Normalize(new FlatVector(dx, dy));
                    FlatVector force = forecdirection * forceMagnitude;
                    body.AddForce(force);  
                }

                if (keyboard.IsKeyDown(Keys.R))
                {
                    body.Rotate(MathF.PI/2f*FlatUtil.GetElapsedTimeInSeconds(gameTime));
                }

#endif
            }

            if (sampleTimer.Elapsed.TotalSeconds > 1d)
            {
                bodyCountString = "BodyCount: "+Math.Round(this.totalBodyCount /(double)this.totalSampleCount,4).ToString();
                worldStepTimeString = "StepTime: "+Math.Round(totalWorldStepTime / (double)totalSampleCount,4).ToString();
                totalBodyCount = 0;
                totalSampleCount = 0;
                totalWorldStepTime = 0;
                sampleTimer.Restart();
            }

            watch.Restart();    
            this.world.Step(FlatUtil.GetElapsedTimeInSeconds(gameTime),20);
            watch.Stop();

            totalWorldStepTime += watch.Elapsed.TotalMilliseconds;
            totalBodyCount += world.BodyCount;
            totalSampleCount++;

            //    WrapScreen();

            this.camera.GetExtents(out _, out _, out float viewBottom, out _);

            this.entityRemovalList.Clear();

            for (int i = 0; i < entityList.Count; i++)
            {
                FlatEntity entity = entityList[i];
                FlatBody body = entity.Body;

                if (body.isStatic) { continue; }

                FlatAABB box = body.GetAABB();

                if (box.Max.Y < viewBottom)
                {
                    entityRemovalList.Add(entity);

                }
            }

            for (int i = 0; i < entityRemovalList.Count; i++) {
                FlatEntity entity = entityRemovalList[i];
                world.RemoveBody(entity.Body);
                entityList.Remove(entity);
            }


            base.Update(gameTime);
        }

        protected override void Draw(GameTime gameTime)
        {

            screen.Set();
            GraphicsDevice.Clear(new Color(50, 60, 70));

            
            shapes.Begin(camera);

  
            for(int i=0;i<entityList.Count;i++)    

            {
                entityList[i].Draw(shapes);                    
            }
            
            

            //List<FlatVector> contactPoints = world?.contactPointList;

            //for (int i = 0; i < contactPoints.Count; i++)
            //{
            //    Vector2 contactPosition = FlatConverter.ToVector2(contactPoints[i]);

            //    shapes.DrawBoxFill(contactPosition,0.5f,0.5f,Color.Red);
            //    shapes.DrawBox(contactPosition, 0.5f, 0.5f, Color.White);
            //}

            shapes.End();

            sprites.Begin();

            Vector2 stringSize = fontConsolas18.MeasureString(bodyCountString);


            sprites.DrawString(fontConsolas18, bodyCountString, new Vector2(0, 0), Color.White);
            sprites.DrawString(fontConsolas18, worldStepTimeString, new Vector2(0, stringSize.Y), Color.White);


            sprites.End();

            screen.Unset();
            screen.Present(sprites);




            base.Draw(gameTime);
        }

        private void WrapScreen()
        {
            camera.GetExtents(out Vector2 camMin, out Vector2 camMax);

            float viewWidth = camMax.X- camMin.X;
            float viewHeight = camMax.Y- camMin.Y;

            for (int i = 0; i < world.BodyCount; i++)
            {
                if (!world.GetBody(i, out FlatBody body))
                {
                    throw new Exception();
                }

                if (body.Position.X < camMin.X)
                {
                    body.MoveTo(body.Position + new FlatVector(viewWidth,0f));
                }

                if (body.Position.X > camMax.X)
                {
                    body.MoveTo(body.Position - new FlatVector(viewWidth, 0f));
                }

                if (body.Position.Y < camMin.Y)
                {
                    body.MoveTo(body.Position + new FlatVector(0f, viewHeight));
                }

                if (body.Position.Y > camMax.Y)
                {
                    body.MoveTo(body.Position - new FlatVector(0f, viewHeight));
                }


            }

        }
    }

}
